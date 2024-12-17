#include "definitions.h"
#include <MotorInterface.h>
#include <ScheduleMicro.h>

MotorInterface motor(10000, 600);
ScheduleMicro scheduler(PRINT_INTERVAL_US, JOB_INTERVAL_US, FAULT_CHECK_INTERVAL_US);

void setup() {
  Serial.begin(115200);
  initialiseMotor();
  initialiseScheduler();
}

void loop() {
  updateMotor();
  updateScheduler();

  if (scheduler.taskReady(PRINT_TASK_ID)) {
    Serial.print("Running ");
    Serial.print(motor.status_variables.running);
    Serial.print(" D ");
    Serial.print(motor.status_variables.direction);
    Serial.print(" PR ");
    Serial.print(motor.status_variables.pulses_remaining);
    Serial.print(" F ");
    Serial.println(motor.status_variables.fault);
  }

  if (scheduler.taskReady(JOB_INTERVAL_TASK_ID)) {
    // Check to see if previous job has completed
    if (motor.status_variables.job_id == 0) {
      motor.command_variables.use_ramping = true;
      motor.command_variables.direction = false;  // true / false
      motor.command_variables.microstep = 16;   // Microstepping mode [1, 2, 4, 8, 16, 32] motor pulses per rev (200) times by this number = pulses required for one revolution 
      motor.command_variables.job_id = 1;
      motor.command_variables.pulses = 10000; // Number of pulses to send to motor
      motor.command_variables.pulse_interval = 0;   // Time in microseconds for one motor pulse, default 1000 if 0 entered here
      motor.command_variables.pulse_on_period = 0;  // Time in microseconds for time hardware pin is HIGH, default 500 microseconds if 0 entered here
      motor.command_variables.ramping_steps = 0;  // Controls ramping up and down of motor speed, default 50 pulses if 0 entered here
      motor.StartJob();
    }
  }
}
