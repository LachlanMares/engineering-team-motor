
void initialiseMotor() {
  motor.Init(DIRECTION_PIN, STEP_PIN, SLEEP_PIN, RESET_PIN, FAULT_PIN, M0_PIN, M1_PIN, M2_PIN, ENABLE_PIN);
  motor.Disable();
  motor.Sleep();
  motor.Reset();
  pinMode(MOTOR_INT_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR_INT_PIN_B, INPUT_PULLUP);
  pinMode(MOTOR_INT_PIN_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR_INT_PIN_A), motor_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_INT_PIN_B), motor_encoder_interrupt, CHANGE);
}

void motor_encoder_interrupt() {
  motor.interruptUpdateABExternal(digitalRead(MOTOR_INT_PIN_A), digitalRead(MOTOR_INT_PIN_B), digitalRead(MOTOR_INT_PIN_Z));
}

void updateMotor() {
  if(motor.Update(micros())) {
    motor.ResetJobId();
  }

  if (scheduler.taskReady(FAULT_CHECK_TASK_ID)) {
    //motor.FaultCheck();
  }
}
