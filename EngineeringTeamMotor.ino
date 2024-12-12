#include <MotorInterface.h>
#include <ScheduleMicro.h>


#define ENCODER_UPDATE_PERIOD_US  10000

#define PRINT_TASK_ID  0
#define PRINT_INTERVAL_US  100000

#define JOB_TASK_ID  1
#define JOB_INTERVAL_US  1000000

#define MOTOR_INT_PIN_A 2
#define MOTOR_INT_PIN_B 3

MotorInterface motor(ENCODER_UPDATE_PERIOD_US);
ScheduleMicro scheduler(PRINT_INTERVAL_US, JOB_INTERVAL_US);

bool job_direction = false;

void setup() {
  Serial.begin(115200);
  initialiseMotor();
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
    Serial.print(" ED ");
    Serial.print(motor.encoder.direction);
    Serial.print(" V ");
    Serial.print(motor.encoder.velocity);
    Serial.print(" C ");
    Serial.println(motor.encoder.count);
  }

  if (scheduler.taskReady(JOB_TASK_ID)) {
    if (motor.status_variables.job_id == 0) {
      motor.command_variables.use_ramping = true;
      motor.command_variables.direction = job_direction;
      motor.command_variables.microstep = 1;
      motor.command_variables.job_id = 1;
      motor.command_variables.pulses = 200;
      motor.command_variables.pulse_interval = 0;
      motor.command_variables.pulse_on_period = 0;
      motor.command_variables.ramping_steps = 0;
      motor.StartJob();
      job_direction = !job_direction;
      Serial.println("New Job");
    }
  }
}
