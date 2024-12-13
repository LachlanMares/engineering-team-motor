#include "definitions.h"
#include <MotorInterface.h>
#include <ScheduleMicro.h>
#include <AtSerial.h>

MotorInterface motor(ENCODER_UPDATE_PERIOD_US, ENCODER_PULSES_PER_REVOLUTION);
ScheduleMicro scheduler(PRINT_INTERVAL_US, JOB_INTERVAL_US, STATUS_MESSAGE_INTERVAL_US, MOTOR_FEEDBACK_INTERVAL_US);
AtSerial serialport;

bool job_direction = false;
uint8_t serial_buffer[SERIAL_BUFFER_LENGTH];

void setup() {
  initialiseSerial(); 
  initialiseMotor();
  initialiseScheduler();
}

void loop() {
  updateMotor();
  updateScheduler();
  updateSerial();
  if (scheduler.taskReady(PRINT_TASK_ID)) {
    Serial.print("Running ");
    Serial.print(motor.status_variables.running);
    Serial.print(" D ");
    Serial.print(motor.status_variables.direction);
    Serial.print(" PR ");
    Serial.print(motor.status_variables.pulses_remaining);
    Serial.print(" F ");
    Serial.print(motor.status_variables.fault);
    Serial.print(" ED ");
    Serial.print(motor.encoder.direction);
    Serial.print(" C ");
    Serial.print(motor.encoder.count);
    Serial.print(" AC ");
    Serial.print(motor.encoder.angle_count);
    Serial.print(" VR ");
    Serial.print(motor.encoder.velocity_radians);
    Serial.print(" AR ");
    Serial.println(motor.encoder.angle_radians);
  }

  // if (scheduler.taskReady(JOB_TASK_ID)) {
  //   if (motor.status_variables.job_id == 0) {
  //     motor.command_variables.use_ramping = true;
  //     motor.command_variables.direction = job_direction;
  //     motor.command_variables.microstep = 16;
  //     motor.command_variables.job_id = 1;
  //     motor.command_variables.pulses = 6400;
  //     motor.command_variables.pulse_interval = 0;
  //     motor.command_variables.pulse_on_period = 0;
  //     motor.command_variables.ramping_steps = 400;
  //     motor.StartJob();
  //     job_direction = !job_direction;
  //   }
  // }
}
