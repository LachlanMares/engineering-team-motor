#!/usr/bin/env python3

"""
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    ??

Description:
    ??
"""

import warnings
import struct
import math
import time
import threading
import serial
import queue
import random
from typing import Union
from pathlib import Path
from copy import deepcopy

warnings.filterwarnings("ignore")

from definition_file_parser import parse_definitions_file


class Motor:
    def __init__(self, definitions_filepath: Path, serial_port: Union[str, None] = None):
        """
        Description:

        Args:
            serial_port (str): Default serial port path as string
        """
        # Serial settings
        self.serial_port_name = serial_port

        definitions = parse_definitions_file(definitions_filepath)

        self.baud_rate = definitions['serial_settings']['BAUD_RATE']
        self.byte_STX = struct.pack('!B', definitions['serial_settings']['STX'])

        self.STX = definitions['serial_settings']['STX']
        self.ETX = definitions['serial_settings']['ETX']
        self.ACK = definitions['serial_settings']['ACK']
        self.NAK = definitions['serial_settings']['NAK']

        # Status message
        self.motor_status_dict = {}

        # constants
        self.two_pi = 2 * math.pi

        # encoder_settings
        self.encoder_update_period_us = definitions['encoder_settings']['ENCODER_UPDATE_PERIOD_US']
        self.encoder_pulses_per_revolution = definitions['encoder_settings']['ENCODER_PULSES_PER_REVOLUTION']
        self.encoder_setpoint_tolerance = definitions['encoder_settings']['ENCODER_SETPOINT_TOLERANCE']
        self.radians_per_encoder_pulse = self.two_pi / self.encoder_pulses_per_revolution

        # status_message_bits
        self.status_direction_bit = 1 << definitions['status_message_bits']['STATUS_DIRECTION_BIT']
        self.status_fault_bit = 1 << definitions['status_message_bits']['STATUS_FAULT_BIT']
        self.status_paused_bit = 1 << definitions['status_message_bits']['STATUS_PAUSED_BIT']
        self.status_ramping_bit = 1 << definitions['status_message_bits']['STATUS_RAMPING_BIT']
        # self.status_spare_bit = 1 << definitions['status_message_bits']['STATUS_SPARE_BIT']
        self.status_enabled_bit = 1 << definitions['status_message_bits']['STATUS_ENABLED_BIT']
        self.status_running_bit = 1 << definitions['status_message_bits']['STATUS_RUNNING_BIT']
        self.status_sleep_bit = 1 << definitions['status_message_bits']['STATUS_SLEEP_BIT']

        # message_types
        self.motor_status_message_id = definitions['message_types']['MOTOR_STATUS_MESSAGE_ID']
        self.motor_feedback_message_id = definitions['message_types']['MOTOR_FEEDBACK_MESSAGE_ID']
        self.motor_in_fault_message_id = definitions['message_types']['MOTOR_FAULT_MESSAGE_ID']
        self.response_message_id = definitions['message_types']['RESPONSE_MESSAGE_ID']
        self.job_complete_message_id = definitions['message_types']['JOB_COMPLETE_MESSAGE_ID']
        self.job_cancelled_message_id = definitions['message_types']['JOB_CANCELLED_MESSAGE_ID']

        self.command_dict = definitions['command_types']
        self.response_dict = definitions['response_types']

        self.microsteps = [1, 2, 4, 8, 16, 32]
        self.minimum_pulse_interval_us = definitions['motor_settings']['MINIMUM_PULSE_INTERVAL']
        self.motor_pulses_per_revolution = definitions['motor_settings']['MOTOR_STEPS_PER_REV']
        self.default_pulse_on_period = definitions['motor_settings']['DEFAULT_PULSE_ON_PERIOD']
        self.max_pulses_per_second = 1e6 / self.minimum_pulse_interval_us
        max_motor_rpm = (self.max_pulses_per_second / self.motor_pulses_per_revolution) * 60
        self.max_motor_rpm_list = [max_motor_rpm / j for j in self.microsteps]

        self.job_active = False
        self.job_pending = False
        self.job_response_code = -1
        self.status_job_id = 0
        self.commanded_job_type = 0
        self.requested_job = 0
        self.current_job_id = 0
        self.status_message_dict = {"job_id": 0,
                                    "status": {
                                        "direction" : True,
                                        "fault": False,
                                        "paused": False,
                                        "using_ramping": False,
                                        "enabled": False,
                                        "running": False,
                                        "sleeping": False
                                    },
                                    "microstep": 1,
                                    "pulses_remaining": 0}
        self.at_commanded_position = True
        self.commanded_position = 0.0
        self.commanded_speed = 10.0
        self.current_motor_velocity = 0.0
        self.current_motor_position = 0.0
        self.current_motor_encoder_count = 0

        # Incoming messages don't include header or footer bytes
        self.motor_status_message_struct = struct.Struct('<4BLB')  # {MOTOR_STATUS_MESSAGE_ID, motor.status_byte, motor.status_variables.job_id, motor.status_variables.microstep, motor.status_variables.pulses_remaining, ETX}
        self.motor_feedback_message_struct = struct.Struct('<B2fhB')  # {MOTOR_FEEDBACK_MESSAGE_ID, motor.encoder_status.velocity_radians, motor.encoder.angle_radians, motor.encoder_status.angle_count, ETX}
        self.motor_in_fault_message_struct = struct.Struct('<2B')  # {MOTOR_FAULT_MESSAGE_ID, ETX};
        self.response_message_struct = struct.Struct('<6B')  # {RESPONSE_MESSAGE_ID, COMMAND, JOB_ID, RESPONSE, [ACK or NAK], ETX};
        self.job_complete_message_struct = struct.Struct('<3B')  # {JOB_COMPLETE_MESSAGE_ID, motor.status_variables.job_id, ETX}
        self.job_cancelled_message_struct = struct.Struct('<3B')  # {JOB_CANCELLED_MESSAGE_ID, motor_ptr.status_variables.job_id, ETX};

        self.send_queue = queue.Queue(maxsize=20)
        self.receive_queue = queue.Queue(maxsize=20)
        self.read_thread = None
        self.updating_thread = None
        self.read_lock = threading.Lock()
        self.running = False
        self.ser = None
        self.connected = False

    def start_threads(self):
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

            self.updating_thread = threading.Thread(target=self.processing_loop, daemon=True)
            self.updating_thread.start()


    def stop(self):
        print("\nShutting down motor...")
        self.send_cancel_job()
        self.send_sleep_motor()
        self.send_disable_motor()
        time.sleep(0.5)

        self.running = False
        self.read_thread.join()
        self.updating_thread.join()
        self.ser.close()

        print("Complete")

    def connect_serial_port(self):
        """
        Description:
            Loop until a valid serial connection is found.
        """
        while not self.connected:
            print(f"Trying to connect serial...")

            try:
                self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=2, )
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
                new_message_id = 1

                while new_message_id != 0:
                    new_message_id, serial_buffer = self.get_serial_message()

                    if new_message_id == self.motor_feedback_message_id:
                        feedback_message = self.motor_feedback_message_struct.unpack(serial_buffer)

                        with self.read_lock:
                            self.current_motor_velocity = feedback_message[1]
                            self.current_motor_position = feedback_message[2]
                            self.current_motor_encoder_count = feedback_message[3]

                    else:
                        try:
                            self.receive_queue.put({"id": new_message_id,
                                                    "msg": serial_buffer})
                        except queue.Full:
                            new_message_id = 0
                            print(f'Receive queue is full')

                        except Exception as e:
                            new_message_id = 0
                            print(f'Exception {e}')

                try:
                    new_message = self.send_queue.get(timeout=0.001)
                    self.ser.write(new_message)

                except queue.Empty:
                    pass

                except Exception as e:
                    print(f'Exception {e}')

            else:
                self.ser.close()
                self.connected = False
                self.connect_serial_port()

            time.sleep(0.0001)

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

                serial_buffer = self.ser.read(int.from_bytes(message_length, "little") - 2)

                if serial_buffer[-1] == self.ETX:
                    return serial_buffer[0], serial_buffer

        return 0, []

    def send_motor_rotations_at_set_rpm(self,
                                        number_or_rotations: Union[float, int],
                                        rpm: Union[float, int],
                                        direction: bool,
                                        use_ramping: bool = False,
                                        ramping_steps: int = 0,
                                        ramp_scaler: Union[int, None] = None,
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
                                              pulse_on_period=self.default_pulse_on_period,
                                              use_ramping=use_ramping,
                                              ramping_steps=ramping_steps,
                                              ramp_scaler=ramp_scaler,
                                              job_id=job_id,
                                              )

            else:
                best_step_choice = 1
                for (m_step, max_rpms) in zip(self.microsteps, self.max_motor_rpm_list):
                    if rpm < max_rpms:
                        best_step_choice = m_step

                pulse_interval = int((1 / ((rpm / 60) * self.motor_pulses_per_revolution * best_step_choice)) * 1e6)
                required_pulses = int(abs(number_or_rotations) * self.motor_pulses_per_revolution) * best_step_choice

                # print(f"{pulse_interval=} uS, {required_pulses=}, {best_step_choice=}")

                return self.send_motor_pulses(direction=direction,
                                              microstep=best_step_choice,
                                              pulses=required_pulses,
                                              pulse_interval=pulse_interval,
                                              pulse_on_period=self.default_pulse_on_period,
                                              use_ramping=use_ramping,
                                              ramping_steps=ramping_steps,
                                              ramp_scaler=ramp_scaler,
                                              job_id=job_id,
                                              )

    def send_motor_pulses_at_set_rpm(self,
                                     rpm: Union[float, int],
                                     direction: bool,
                                     pulses: int = 0,
                                     use_ramping: bool = False,
                                     ramping_steps: int = 0,
                                     ramp_scaler: Union[int, None] = None,
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
                                          ramp_scaler=ramp_scaler,
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
                                          pulse_on_period=self.default_pulse_on_period,
                                          use_ramping=use_ramping,
                                          ramping_steps=ramping_steps,
                                          ramp_scaler=ramp_scaler,
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
                             ramp_scaler: Union[int, None] = None,
                             job_id: int = 0,
                             ):

        m_step = microstep if microstep in self.microsteps else 1
        required_pulses = int(abs(number_or_rotations) * self.motor_pulses_per_revolution) * m_step

        self.send_motor_pulses(direction=direction,
                               microstep=m_step,
                               pulses=required_pulses,
                               pulse_interval=pulse_interval,
                               pulse_on_period=pulse_on_period,
                               use_ramping=use_ramping,
                               ramping_steps=ramping_steps,
                               ramp_scaler=ramp_scaler,
                               job_id=job_id,
                               )

    def goto_rotor_position_radians(self,
                                    desired_position: float,
                                    direction: bool,
                                    rpm: Union[float, int],
                                    use_ramping: bool = False,
                                    ramping_steps: int = 0,
                                    ramp_scaler: Union[int, None] = None,
                                    job_id: int = 0,
                                    is_adjustment: bool = False,
                                    ):

        if rpm < 0:
            return -1

        elif rpm > self.max_motor_rpm_list[0]:
            best_step_choice = self.microsteps[0]
            pulse_interval = self.minimum_pulse_interval_us

        else:
            best_step_choice  = 1
            for (m_step, max_rpms) in zip(self.microsteps, self.max_motor_rpm_list):
                if rpm < max_rpms:
                    best_step_choice = m_step

            pulse_interval = int((1 / ((rpm / 60) * self.motor_pulses_per_revolution * best_step_choice)) * 1e6)

        current_motor_position = self.get_rotor_position()

        if not self.motor_is_at_target(desired_position):
            self.at_commanded_position = False
            self.commanded_speed = rpm
            self.commanded_position = desired_position

            if not is_adjustment:
                if direction:
                    if desired_position >= current_motor_position:
                        delta_position = desired_position - current_motor_position
                    else:
                        delta_position = self.two_pi - (current_motor_position - desired_position)
                else:
                    if desired_position <= current_motor_position:
                        delta_position = current_motor_position - desired_position
                    else:
                        delta_position = self.two_pi - (desired_position - current_motor_position)

            else:
                delta_position = abs(desired_position - current_motor_position)
                direction = desired_position >= current_motor_position

            number_or_rotations = delta_position / self.two_pi
            required_pulses = int(number_or_rotations * self.motor_pulses_per_revolution) * best_step_choice

            # print(f"{self.get_rotor_position():.3f}, {self.commanded_position=}, {delta_position=}, {required_pulses=}, {best_step_choice=}")

            if required_pulses > 0:
                self.send_motor_pulses(direction=direction,
                                       microstep=best_step_choice,
                                       pulses=required_pulses,
                                       pulse_interval=pulse_interval,
                                       pulse_on_period=self.default_pulse_on_period,
                                       use_ramping=use_ramping,
                                       ramping_steps=ramping_steps,
                                       ramp_scaler=ramp_scaler,
                                       job_id=job_id,
                                       )
            else:
                print(f"Job with zero pulses requested")
                self.job_active = False
                self.at_commanded_position = True
                return -1

        else:
            print(f"Rotor already at correct position")
            self.job_active = False
            self.at_commanded_position = True
            return -1

    def send_motor_pulses(self,
                          direction: bool,
                          microstep: int = 1,
                          pulses: int = 0,
                          pulse_interval: int = 1000,
                          pulse_on_period: Union[int, None] = None,
                          use_ramping: bool = False,
                          ramping_steps: int = 0,
                          ramp_scaler: Union[int, None] = None,
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
                if ramp_scaler is None:
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
                else:
                    command = self.command_dict['SEND_JOB_ALL_VARIABLES_WITH_RAMPING_AND_RATE']
                    self.send_queue.put(struct.pack('!6B4I2B',
                                                    self.STX,
                                                    24,
                                                    command,
                                                    1 if direction else 0,
                                                    microstep if microstep in self.microsteps else 1,
                                                    job_id,
                                                    pulses,
                                                    pulse_interval if pulse_interval > self.minimum_pulse_interval_us else self.minimum_pulse_interval_us,
                                                    pulse_on_period,
                                                    ramping_steps,
                                                    ramp_scaler,
                                                    self.ETX
                                                    ))

        self.job_active = False
        self.job_pending = True
        self.requested_job = job_id
        self.commanded_job_type = command

    def get_rotor_position(self):
        with self.read_lock:
            position = deepcopy(self.current_motor_position)
        return position

    def send_pause_job(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['PAUSE_JOB'],
                                        self.ETX
                                        ))

    def send_resume_job(self):
        self.send_queue.put(struct.pack('<4B',
                            self.STX,
                            4,
                            self.command_dict['RESUME_JOB'],
                            self.ETX
                            ))

    def send_cancel_job(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['CANCEL_JOB'],
                                        self.ETX
                                        ))

    def send_enable_motor(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['ENABLE_MOTOR'],
                                        self.ETX
                                        ))

    def send_disable_motor(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['DISABLE_MOTOR'],
                                        self.ETX
                                        ))

    def send_sleep_motor(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['SLEEP_MOTOR'],
                                        self.ETX
                                        ))

    def send_wake_motor(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['WAKE_MOTOR'],
                                        self.ETX
                                        ))

    def send_reset_motor(self):
        self.send_queue.put(struct.pack('<4B',
                                        self.STX,
                                        4,
                                        self.command_dict['RESET_MOTOR'],
                                        self.ETX
                                        ))

    def motor_is_at_target(self, desired_position):
        current_motor_position = self.get_rotor_position()

        return abs(current_motor_position - desired_position) < self.encoder_setpoint_tolerance * self.radians_per_encoder_pulse

    def process_status_message(self, status_message: Union[list, tuple]):
        self.status_message_dict = {"job_id": status_message[2],
                                    "status": {
                                        "direction": (status_message[1] & self.status_direction_bit) > 0,
                                        "fault": (status_message[1] & self.status_fault_bit) > 0,
                                        "paused": (status_message[1] & self.status_paused_bit) > 0,
                                        "using_ramping": (status_message[1] & self.status_ramping_bit) > 0,
                                        "enabled": (status_message[1] & self.status_enabled_bit) > 0,
                                        "running": (status_message[1] & self.status_running_bit) > 0,
                                        "sleeping": (status_message[1] & self.status_sleep_bit) > 0,
                                    },
                                    "microstep": status_message[3],
                                    "pulses_remaining": status_message[4]
                                    }

    def processing_loop(self):
        while self.running:
            try:
                new_message_dict = self.receive_queue.get(timeout=0.01)

                if new_message_dict["id"] == self.motor_status_message_id:
                    self.process_status_message(status_message=self.motor_status_message_struct.unpack(new_message_dict["msg"]))

                elif new_message_dict["id"] == self.motor_in_fault_message_id:
                    fault_message = self.motor_in_fault_message_struct.unpack(new_message_dict["msg"])
                    print(f"Motor Fault")

                elif new_message_dict["id"] == self.response_message_id:
                    response_message = self.response_message_struct.unpack(new_message_dict["msg"])
                    # print(f"{response_message=}")
                    if response_message[1] == self.commanded_job_type:
                        self.commanded_job_type = 0

                        if response_message[2] == self.requested_job and response_message[4] == self.ACK:
                            self.current_job_id = self.requested_job
                            self.job_active = True
                            self.job_pending = False

                        else:
                            self.current_job_id = 0
                            self.requested_job = 0
                            self.job_active = False
                            self.job_pending = False

                elif new_message_dict["id"] == self.job_complete_message_id:
                    job_complete_message = self.job_complete_message_struct.unpack(new_message_dict["msg"])

                    if job_complete_message[1] == self.current_job_id:

                        if not self.at_commanded_position:
                            if self.motor_is_at_target(self.commanded_position):
                                self.at_commanded_position = True
                                self.job_active = False
                                self.send_sleep_motor()

                            else:
                                self.goto_rotor_position_radians(desired_position=self.commanded_position,
                                                                 direction=True,
                                                                 rpm=self.commanded_speed,
                                                                 job_id=self.current_job_id,
                                                                 is_adjustment=True)
                        else:
                            self.at_commanded_position = True
                            self.job_active = False
                            self.send_sleep_motor()

                elif new_message_dict["id"] == self.job_cancelled_message_id:
                    job_cancelled_message = self.job_cancelled_message_struct.unpack(new_message_dict["msg"])

                    if job_cancelled_message[1] == self.current_job_id:
                        self.job_active = False

            except queue.Empty:
                time.sleep(0.005)

            except Exception as e:
                print(f'Exception {e}')

    def is_ready_for_job(self):
        return not self.job_active and not self.job_pending


if __name__ == "__main__":
    project_dir = Path(__file__).resolve().parents[1]
    header_file = project_dir / 'arduino/engineering-team-motor/definitions.h'

    motor = Motor(definitions_filepath=header_file, serial_port='/dev/arduino_rp2040')
    motor.start_threads()
    motor.send_enable_motor()
    motor.send_wake_motor()

    try:
        while True:
            if motor.is_ready_for_job():
                time.sleep(1)
                motor.send_wake_motor()
                motor.send_motor_rotations_at_set_rpm(number_or_rotations=1,
                                                      rpm=random.random() * 10,
                                                      direction=random.choice([True, False]),
                                                      job_id=1)

                # motor.send_motor_rotations(number_or_rotations=2,
                #                            direction=random.choice([True, False]),
                #                            microstep=32,
                #                            pulse_interval=20000,
                #                            pulse_on_period=500,
                #                            use_ramping=True,
                #                            ramping_steps=250,
                #                            ramp_scaler=3,
                #                            job_id=1)
                #
                # motor.goto_rotor_position_radians(desired_position=random.choice([math.pi/2, math.pi, 1.5*math.pi]),
                #                                   direction=random.choice([True, False]),
                #                                   rpm=10.0,
                #                                   use_ramping=True,
                #                                   ramping_steps=100,
                #                                   ramp_scaler=5,
                #                                   job_id=1)

            else:
                time.sleep(0.5)
                print(f"{motor.get_rotor_position()=:.3f} radians")
                print(motor.status_message_dict)

    except KeyboardInterrupt:
        motor.stop()

