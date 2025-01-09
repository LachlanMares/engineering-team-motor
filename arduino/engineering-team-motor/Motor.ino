
void initialiseMotor() {
  motor.Init(DIRECTION_PIN, STEP_PIN, SLEEP_PIN, RESET_PIN, FAULT_PIN, M0_PIN, M1_PIN, M2_PIN, ENABLE_PIN);

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
    uint8_t job_complete_buffer[JOB_COMPLETE_MESSAGE_LENGTH] = {JOB_COMPLETE_MESSAGE_ID, motor.status_variables.job_id};
    serialport.sendMessage(&job_complete_buffer[0], JOB_COMPLETE_MESSAGE_LENGTH);
    motor.ResetJobId();
  }

  if (scheduler.taskReady(FAULT_CHECK_TASK_ID)) {
    if (motor.FaultStatus()) {
      uint8_t motor_in_fault_buffer[MOTOR_FAULT_MESSAGE_LENGTH] = {MOTOR_FAULT_MESSAGE_ID};
      serialport.sendMessage(&motor_in_fault_buffer[0], MOTOR_FAULT_MESSAGE_LENGTH);
    }
  }
}
