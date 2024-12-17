
void initialiseMotor() {
  motor.Init(DIRECTION_PIN, STEP_PIN, SLEEP_PIN, RESET_PIN, FAULT_PIN, M0_PIN, M1_PIN, M2_PIN, ENABLE_PIN);
  motor.Enable();
  motor.Sleep();
}

void updateMotor() {
  if(motor.Update(micros())) {
    motor.ResetJobId();
  }

  if (scheduler.taskReady(FAULT_CHECK_TASK_ID)) {
    motor.FaultCheck();
  }
}
