#include "definitions.h"
#include <MotorInterface.h>
#include <ScheduleMicro.h>
#include <AtSerial.h>

MotorInterface motor(ENCODER_UPDATE_PERIOD_US, ENCODER_PULSES_PER_REVOLUTION, true);
ScheduleMicro scheduler(FAULT_CHECK_INTERVAL_US, STATUS_MESSAGE_INTERVAL_US, MOTOR_FEEDBACK_INTERVAL_US);
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
}
