#!/usr/bin/env python3

import warnings
import struct
import time
import threading
import serial
import queue
from typing import Union
from copy import deepcopy

warnings.filterwarnings("ignore")


class Motor:
    def __init__(self, definitions: dict, serial_port: Union[str, None] = None):
        """
        Description:

        Args:
            serial_port (str): Default serial port path as string
            definitions (dict): Parsed header file used by the Arduino
        """
        # Serial settings
        self.serial_port_name = serial_port
        self.BAUD_RATE = definitions['serial_settings']['BAUD_RATE']
        self.byte_STX = struct.pack('!B', definitions['serial_settings']['STX'])

        self.STX = definitions['serial_settings']['STX']
        self.ETX = definitions['serial_settings']['ETX']
        self.ACK = definitions['serial_settings']['ACK']
        self.NAK = definitions['NAK']

        # Status message
        self.motor_status_dict = {}

        # encoder_settings
        self.encoder_update_period_us = definitions['encoder_settings']['ENCODER_UPDATE_PERIOD_US']
        self.encoder_pulses_per_revolution = definitions['encoder_settings']['ENCODER_PULSES_PER_REVOLUTION']

        # status_message_bits
        self.status_direction_bit = definitions['status_message_bits']['STATUS_DIRECTION_BIT']
        self.status_fault_bit = definitions['status_message_bits']['STATUS_FAULT_BIT']
        self.status_paused_bit = definitions['status_message_bits']['STATUS_PAUSED_BIT']
        self.status_ramping_bit = definitions['status_message_bits']['STATUS_RAMPING_BIT']
        # self.status_spare_bit = definitions['status_message_bits']['STATUS_SPARE_BIT']
        self.status_enabled_bit = definitions['status_message_bits']['STATUS_ENABLED_BIT']
        self.status_running_bit = definitions['status_message_bits']['STATUS_RUNNING_BIT']
        self.status_sleep_bit = definitions['status_message_bits']['STATUS_SLEEP_BIT']

        # message_types
        self.motor_status_message_id = definitions['message_types']['MOTOR_STATUS_MESSAGE_ID']
        self.motor_feedback_message_id = definitions['message_types']['MOTOR_FEEDBACK_MESSAGE_ID']
        self.response_message_id = definitions['message_types']['RESPONSE_MESSAGE_ID']
        self.job_complete_message_id = definitions['message_types']['JOB_COMPLETE_MESSAGE_ID']
        self.job_cancelled_message_id = definitions['message_types']['JOB_CANCELLED_MESSAGE_ID']

        self.command_dict = definitions['command_types']

        self.microsteps = [1, 2, 4, 8, 16, 32]
        self.minimum_pulse_interval_us = definitions['motor']['MINIMUM_PULSE_INTERVAL_US']
        self.motor_pulses_per_revolution = definitions['motor']['MOTOR_STEPS_PER_REV']
        self.max_pulses_per_second = 1e6 / self.minimum_pulse_interval_us
        max_motor_rpm = (self.max_pulses_per_second / self.motor_pulses_per_revolution) * 60
        self.max_motor_rpm_list = [max_motor_rpm / j for j in self.microsteps]

        self.current_motor_velocity = 0.0
        self.current_motor_position = 0.0

        # Incoming messages don't include header or footer bytes
        self.motor_status_message_struct = struct.Struct('!4BL')  # {MOTOR_STATUS_MESSAGE_ID, motor.status_byte, motor.status_variables.job_id, motor.status_variables.microstep, motor.status_variables.pulses_remaining}
        self.motor_feedback_message_struct = struct.Struct('!B2f')  # {MOTOR_FEEDBACK_MESSAGE_ID, motor.encoder.velocity_radians, motor.encoder.angle_radians}
        self.response_message_struct = struct.Struct('!4B')  # {RESPONSE_MESSAGE_ID, COMMAND, RESPONSE, [ACK or NAK]};
        self.job_complete_message_struct = struct.Struct('!2B')  # {JOB_COMPLETE_MESSAGE_ID, motor.status_variables.job_id}
        self.job_cancelled_message_struct = struct.Struct('!2B')  # {JOB_CANCELLED_MESSAGE_ID, motor_ptr.status_variables.job_id};

        self.send_queue = queue.Queue(maxsize=20)
        self.receive_queue = queue.Queue(maxsize=20)
        self.read_thread = None
        self.updating_thread = None
        self.read_lock = threading.Lock()
        self.running = False      
        self.ser = None
        self.connected = False

    def start_thread(self):
        """
        Description:
            Starts serial communications in a separate thread.

        Args:

        """
        self.connect_serial_port()

        if self.ser is not None:
            self.running = True
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

            self.updating_thread = threading.Thread(target=self.serial_write_loop, daemon=True)
            self.updating_thread.start()


    def stop(self):
        self.running = False
        self.read_thread.join()
        self.updating_thread.join()
        self.ser.close()

    def connect_serial_port(self):
        """
        Description:
            Loop until a valid serial connection is found.
        """
        while not self.connected:
            print(f"Trying to connect serial")

            try:
                self.ser = serial.Serial(self.serial_port_name, self.BAUD_RATE, timeout=2, )
                if self.ser.isOpen():
                    self.connected = True
                    print(f"Serial connected")
                    break
                else:
                    time.sleep(1)
                    print(f"Serial timeout")

            except:
                pass

    def serial_read_loop(self):
        """
        Description:
            Serial communications are handled in a separate thread.
            Publishes messages from stored lists
            Receives messages, checks validity and stores in lists

        Args:

        Returns:

        """
        while self.running:
            # A check to see if serial port is open
            if self.ser.isOpen():
                new_message_id, serial_buffer = self.get_serial_message()

                if new_message_id != 0:
                    if new_message_id == self.motor_feedback_message_id:
                        feedback_message = self.motor_feedback_message_struct.unpack(serial_buffer)
                        self.current_motor_velocity = feedback_message[1]
                        self.current_motor_position = feedback_message[2]

                    else:
                        try:
                            self.receive_queue.put({"id": new_message_id, 
                                                    "msg": serial_buffer})    
                        except queue.Full:
                            print(f'Receive queue is full')

                        except Exception as e:
                            print(f'Exception {e}')

                try:
                    new_message = self.send_queue.get(timeout=0.01)
                    self.ser.write(new_message)

                except queue.Empty:
                    pass

                except Exception as e:
                    print(f'Exception {e}')

            else:
                self.ser.close()
                self.connected = False
                self.connect_serial_port()

        self.ser.close()

    def get_serial_message(self):
        """
        Description:
            Checks if serial buffer contains new data, looks for standard AT protocol message.
            Valid message has the following byte structure STX, length, data, ETX. data[0] is the message id

        Returns:
            message id: (int) message identifier
            message buffer: (bytes) message contents
        """
        if self.ser.inWaiting() != 0:
            if self.ser.read() == self.byte_STX:
                message_length = self.ser.read()
                serial_buffer = self.ser.read(int.from_bytes(message_length, "big") - 2)

                if serial_buffer[-1] == self.ETX:
                    return serial_buffer[0], serial_buffer

        return 0, []

    def send_motor_rotations_at_set_rpm(self,
                                        number_or_rotations: Union[float, int],
                                        rpm: Union[float, int],
                                        direction: bool,
                                        use_ramping: bool = False,
                                        ramping_steps: int = 0,
                                        job_id: int = 0,
                                        ):

            if rpm < 0:
                return -1

            elif rpm > self.max_motor_rpm_list[0]:
                required_pulses = int(abs(number_or_rotations) * self.motor_pulses_per_revolution)
                return self.send_motor_pulses(direction=direction,
                                              microstep=1,
                                              pulses=required_pulses,
                                              pulse_interval=self.minimum_pulse_interval_us,
                                              pulse_on_period=None,
                                              use_ramping=use_ramping,
                                              ramping_steps=ramping_steps,
                                              job_id=job_id,
                                              )

            else:
                best_step_choice = 1
                for (m_step, max_rpms) in zip(self.microsteps, self.max_motor_rpm_list):
                    if rpm < max_rpms:
                        best_step_choice = m_step

                pulse_interval = int((1 / ((rpm / 60) * self.motor_pulses_per_revolution * best_step_choice)) * 1e6)
                required_pulses = int(abs(number_or_rotations) * self.motor_pulses_per_revolution) * best_step_choice

                return self.send_motor_pulses(direction=direction,
                                              microstep=best_step_choice,
                                              pulses=required_pulses,
                                              pulse_interval=pulse_interval,
                                              pulse_on_period=None,
                                              use_ramping=use_ramping,
                                              ramping_steps=ramping_steps,
                                              job_id=job_id,
                                              )

    def send_motor_pulses_at_set_rpm(self,
                                     rpm: Union[float, int],
                                     direction: bool,
                                     pulses: int = 0,
                                     use_ramping: bool = False,
                                     ramping_steps: int = 0,
                                     job_id: int = 0,
                                     ):

        if rpm < 0:
            return -1

        elif rpm > self.max_motor_rpm_list[0]:
            return self.send_motor_pulses(direction=direction,
                                          microstep=1,
                                          pulses=pulses,
                                          pulse_interval=self.minimum_pulse_interval_us,
                                          use_ramping=use_ramping,
                                          ramping_steps=ramping_steps,
                                          job_id=job_id,
                                          )

        else:
            best_step_choice = 1
            for (m_step, max_rpms) in zip(self.microsteps, self.max_motor_rpm_list):
                if rpm < max_rpms:
                    best_step_choice = m_step

            pulse_interval = int((1 / ((rpm / 60) * self.motor_pulses_per_revolution * best_step_choice)) * 1e6)

            return self.send_motor_pulses(direction=direction,
                                          microstep=best_step_choice,
                                          pulses=pulses,
                                          pulse_interval=pulse_interval,
                                          pulse_on_period=None,
                                          use_ramping=use_ramping,
                                          ramping_steps=ramping_steps,
                                          job_id=job_id,
                                          )

    def send_motor_rotations(self,
                             number_or_rotations: Union[float, int],
                             direction: bool,
                             microstep: int = 1,
                             pulse_interval: int = 1000,
                             pulse_on_period: Union[int, None] = None,
                             use_ramping: bool = False,
                             ramping_steps: int = 0,
                             job_id: int = 0,
                             ):

        m_step = microstep if microstep in self.microsteps else 1
        required_pulses = int(abs(number_or_rotations) * self.motor_pulses_per_revolution) * m_step

        return self.send_motor_pulses(direction=direction,
                                      microstep=m_step,
                                      pulses=required_pulses,
                                      pulse_interval=pulse_interval,
                                      pulse_on_period=pulse_on_period,
                                      use_ramping=use_ramping,
                                      ramping_steps=ramping_steps,
                                      job_id=job_id,
                                      )

    def send_motor_pulses(self,
                          direction: bool,
                          microstep: int = 1,
                          pulses: int = 0,
                          pulse_interval: int = 1000,
                          pulse_on_period: Union[int, None] = None,
                          use_ramping: bool = False,
                          ramping_steps: int = 0,
                          job_id: int = 0,
                          **kwargs):

        if pulse_on_period is None:  # Basic send job
            if not use_ramping:
                command = self.command_dict['SEND_JOB']
                self.send_queue.put(struct.pack('!6BIB',
                                                self.STX,
                                                11,
                                                command,
                                                1 if direction else 0,
                                                microstep if microstep in self.microsteps else 1,
                                                job_id,
                                                pulses,
                                                self.ETX
                                                ))
            else:
                command = self.command_dict['SEND_JOB_WITH_RAMPING']
                self.send_queue.put(struct.pack('!6B2IB',
                                                self.STX,
                                                15,
                                                command,
                                                1 if direction else 0,
                                                microstep if microstep in self.microsteps else 1,
                                                job_id,
                                                pulses,
                                                ramping_steps,
                                                self.ETX
                                                )) 
        else:
            if not use_ramping:
                command = self.command_dict['SEND_JOB_ALL_VARIABLES']
                self.send_queue.put(struct.pack('!6B3IB',
                                                self.STX,
                                                19,
                                                command,
                                                1 if direction else 0,
                                                microstep if microstep in self.microsteps else 1,
                                                job_id,
                                                pulses,
                                                pulse_interval if pulse_interval >self.minimum_pulse_interval_us else self.minimum_pulse_interval_us,
                                                pulse_on_period,
                                                self.ETX
                                                ))
            else:
                command = self.command_dict['SEND_JOB_ALL_VARIABLES_WITH_RAMPING']
                self.send_queue.put(struct.pack('!6B4IB',
                                                self.STX,
                                                23,
                                                command,
                                                1 if direction else 0,
                                                microstep if microstep in self.microsteps else 1,
                                                job_id,
                                                pulses,
                                                pulse_interval if pulse_interval > self.minimum_pulse_interval_us else self.minimum_pulse_interval_us,
                                                pulse_on_period,
                                                ramping_steps,
                                                self.ETX
                                                ))

        return command


    def send_pause_job(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['PAUSE_JOB'],
                                        self.ETX
                                        ))

    def send_resume_job(self):
        self.send_queue.put(struct.pack('!4B',
                            self.STX,
                            4,
                            self.command_dict['RESUME_JOB'],
                            self.ETX
                            ))

    def send_cancel_job(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['CANCEL_JOB'],
                                        self.ETX
                                        ))


    def send_enable_motor(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['ENABLE_MOTOR'],
                                        self.ETX
                                        ))

    def send_disable_motor(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['DISABLE_MOTOR'],
                                        self.ETX
                                        ))

    def send_sleep_motor(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['SLEEP_MOTOR'],
                                        self.ETX
                                        ))

    def send_wake_motor(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['WAKE_MOTOR'],
                                        self.ETX
                                        ))

    def send_reset_motor(self):
        self.send_queue.put(struct.pack('!4B',
                                        self.STX,
                                        4,
                                        self.command_dict['RESET_MOTOR'],
                                        self.ETX
                                        ))

    def get_response_messages(self):
        """
        Description:
            Decode response messages. Every sent message will get either positive or negative acknowledgement.
            response message should be 6 bytes [Response ID, Motor Num, Command, Response, ACK/NAK, ETX]
        Returns:
            response_messages (list): list of response messages
        """
        response_messages = []

        if self.response_message_exists():
            response_messages = self.copy_response_messages()

        for msg in response_messages:
            if len(msg) == 6:
                if msg[4] == self.NAK:
                    print(f"doser-bot nak: {self.get_motor_key(val=msg[1])} {self.get_command_key(val=msg[2])} {self.get_response_key(val=msg[3])}")
                    # todo something

                elif msg[4] == self.ACK:
                    print(f"doser-bot ack: {self.get_motor_key(val=msg[1])} {self.get_command_key(val=msg[2])} {self.get_response_key(val=msg[3])}")
                    # todo something

                else:
                    print(f"unknown ack/nak signal")
            else:
                print(f"invalid response message")

        return response_messages

    def get_job_complete_messages(self):
        """ """
        if self.job_complete_message_exists():
            return self.copy_job_complete_messages()
        else:
            return []

    def get_job_cancelled_messages(self):
        """ """
        if self.job_cancelled_message_exists():
            return self.copy_job_cancelled_messages()

        else:
            return []

    def update_status_variables(self):
        while self.running:
            if self.get_latest_motor_status_message():
                # todo something
                pass
            if self.get_latest_motor_pulses_message():
                # todo something
                pass
            if self.get_latest_load_cell_status_message():
                # todo something
                pass
            time.sleep(0.05)

    def get_latest_motor_status_message(self):
        """
        Description:
            Decode status messages. Status messages are sent on set frequency

        Returns:
            return (dict): Dictionary with current variables
        """

        if self.motor_status_message_exists():
            current_status = self.copy_motor_status_messages()[-1]

            for i, motor_id in enumerate(self.motor_ids_in_use):
                self.motor_status_dict[f'motor_{motor_id}_status'] = current_status[(i * 2) + 1]
                self.motor_status_dict[f'motor_{motor_id}_job_id'] = current_status[(i * 2) + 2]
            return True

        else:
            return False

    def get_latest_motor_pulses_message(self):
        """
        Description:
            Decode status messages. Status messages are sent on set frequency

        Returns:
            return (dict): Dictionary with current variables
        """

        if self.motor_pulses_message_exists():
            current_status = self.copy_motor_pulses_messages()[-1]

            for i, motor_id in enumerate(self.motor_ids_in_use):
                self.motor_status_dict[f'motor_{motor_id}_pulses_remaining'] = current_status[i + 1]

            return True

        else:
            return False

    def get_latest_load_cell_status_message(self):
        """
        Description:
            Decode status messages. Status messages are sent on set frequency

        Returns:
            return (dict): Dictionary with current variables
        """

        if self.load_cell_status_message_exists():
            current_status = self.copy_load_cell_status_messages()[-1]

            for i, load_cell_id in enumerate(self.load_cell_ids_in_use):
                self.load_cell_status_dict[f'load_cell_{load_cell_id}_value'] = current_status[i + 1]

            return True

        else:
            return False

    def get_status_bit_key(self, val):
        for key, value in self.definitions_dict['status_message_bits'].items():
            if val == value:
                return key
        return '??'

    def decode_motor_status(self, motor_status_byte: int):
        return {self.get_status_bit_key(0): (motor_status_byte & 0b00000001) > 0,
                self.get_status_bit_key(1): (motor_status_byte & 0b00000010) > 0,
                self.get_status_bit_key(2): (motor_status_byte & 0b00000100) > 0,
                self.get_status_bit_key(3): (motor_status_byte & 0b00001000) > 0,
                self.get_status_bit_key(4): (motor_status_byte & 0b00010000) > 0,
                self.get_status_bit_key(5): (motor_status_byte & 0b00100000) > 0,
                self.get_status_bit_key(6): (motor_status_byte & 0b01000000) > 0,
                self.get_status_bit_key(7): (motor_status_byte & 0b10000000) > 0,
                }
