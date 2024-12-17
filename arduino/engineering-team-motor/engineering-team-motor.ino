#include "definitions.h"
#include <MotorInterface.h>
#include <ScheduleMicro.h>
#include <AtSerial.h>

MotorInterface motor(ENCODER_UPDATE_PERIOD_US, ENCODER_PULSES_PER_REVOLUTION);
ScheduleMicro scheduler(PRINT_INTERVAL_US, FAULT_CHECK_INTERVAL_US, STATUS_MESSAGE_INTERVAL_US, MOTOR_FEEDBACK_INTERVAL_US);
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


}
