
void initialiseMotor() {
  motor.Enable();
  motor.Sleep();
  
  pinMode(MOTOR_INT_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR_INT_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR_INT_PIN_A), motor_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_INT_PIN_B), motor_encoder_interrupt, CHANGE);
}

void motor_encoder_interrupt() {
  motor.interruptUpdateABExternal(digitalRead(MOTOR_INT_PIN_A), digitalRead(MOTOR_INT_PIN_B));
}

void updateMotor() {
  if(motor.Update(micros())) {
    uint8_t job_complete_buffer[2] = {JOB_COMPLETE_MESSAGE_ID, motor.status_variables.job_id};
    serialport.sendMessage(&job_complete_buffer[0], 2);
    motor.ResetJobId();
  }
}
