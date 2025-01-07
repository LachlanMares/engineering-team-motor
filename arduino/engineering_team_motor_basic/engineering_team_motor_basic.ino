#include "definitions.h"
#include <MotorInterface.h>
#include <ScheduleMicro.h>

MotorInterface motor(ENCODER_UPDATE_PERIOD_US, ENCODER_PULSES_PER_REVOLUTION, true);
ScheduleMicro scheduler(PRINT_INTERVAL_US, FAULT_CHECK_INTERVAL_US, JOB_INTERVAL_INTERVAL_US, MOTOR_FEEDBACK_INTERVAL_US);
int job_counter = 0;

void setup() {
  Serial.begin(115200);
  initialiseMotor();
  initialiseScheduler();
}

void loop() {
  updateMotor();
  updateScheduler();

  if (scheduler.taskReady(PRINT_TASK_ID)) {
    // Serial.print("Running ");
    // Serial.print(motor.status_variables.running);
    // Serial.print(" D ");
    // Serial.print(motor.status_variables.direction);
    // Serial.print(" PR ");
    // Serial.print(motor.status_variables.pulses_remaining);
    // Serial.print(" F ");
    // Serial.print(motor.status_variables.fault);
    // Serial.print(" AC ");
    // Serial.print(motor.encoder_status.angle_count);
    // Serial.print(" VR ");
    // Serial.print(motor.encoder_status.velocity_radians);
    // Serial.print(" AR ");
    // Serial.println(motor.getEncoderAngleRadians());
  }

  if (scheduler.taskReady(JOB_INTERVAL_TASK_ID)) {
    // Check to see if previous job has completed
    if (motor.status_variables.job_id == 0) {
      delay(2000);
      motor.command_variables.use_ramping = false;
      motor.command_variables.direction = true;  // true / false
      motor.command_variables.microstep = 1;   // Microstepping mode [1, 2, 4, 8, 16, 32] motor pulses per rev (200) times by this number = pulses required for one revolution 
      motor.command_variables.job_id = 1;

      if (job_counter < 2) {
        motor.command_variables.microstep = 1;
        motor.command_variables.pulses = 1000;
        motor.command_variables.pulse_interval = 3000; 
        motor.command_variables.pulse_on_period = 1000;  
        motor.command_variables.ramping_steps = 100;  

      } else {
        motor.command_variables.pulses = 1000;
        motor.command_variables.pulse_interval = 1000 + job_counter * 100;   // Time in microseconds for one motor pulse, default 3000 if 0 entered here
        motor.command_variables.pulse_on_period = 1000;  // Time in microseconds for time hardware pin is HIGH, default 500 microseconds if 0 entered here
        motor.command_variables.ramping_steps = 100;  // Controls ramping up and down of motor speed, default 50 pulses if 0 entered here
      }

      motor.StartJob();
      job_counter++;
    }
  }
}
