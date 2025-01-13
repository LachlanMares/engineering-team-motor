void initialiseSerial() {
  serialport.setInitial(BAUD_RATE, SERIAL_TIMEOUT_MS);
}

void updateSerial() {
  int bytes_read = serialport.update(&serial_buffer[0]);

  if(bytes_read > 0) {
    processCommandMessage(&motor, bytes_read);
  }

  if (scheduler.taskReady(STATUS_MESSAGE_TASK_ID)) {
    motor.UpdateStatus();

    uint8_t motor_status_buffer[MOTOR_STATUS_MESSAGE_LENGTH];
    motor_status_buffer[0] = MOTOR_STATUS_MESSAGE_ID;
    motor_status_buffer[1] = motor.status_byte;
    motor_status_buffer[2] = motor.status_variables.job_id; 
    motor_status_buffer[3] = motor.status_variables.microstep;

    memcpy(&motor_status_buffer[4], &motor.status_variables.pulses_remaining, sizeof(unsigned long));

    serialport.sendMessage(&motor_status_buffer[0], MOTOR_STATUS_MESSAGE_LENGTH);
  }

  if (scheduler.taskReady(MOTOR_FEEDBACK_TASK_ID)) {
    uint8_t motor_feedback_buffer[MOTOR_FEEDBACK_MESSAGE_LENGTH];
    motor_feedback_buffer[0] = MOTOR_FEEDBACK_MESSAGE_ID;

    float angle_radians = motor.getEncoderAngleRadians();

    memcpy(&motor_feedback_buffer[1], &motor.encoder_status.velocity_radians, sizeof(float));
    memcpy(&motor_feedback_buffer[5], &angle_radians, sizeof(float));
    memcpy(&motor_feedback_buffer[9], &motor.encoder_status.angle_count, sizeof(int));
    
    serialport.sendMessage(&motor_feedback_buffer[0], MOTOR_FEEDBACK_MESSAGE_LENGTH);
  }
}

void processCommandMessage(MotorInterface* motor_ptr, int bytes_read) {
  uint8_t response_buffer[RESPONSE_MESSAGE_LENGTH] = {RESPONSE_MESSAGE_ID, serial_buffer[0], 0x00, UNKNOWN_MOTOR_COMMAND_RESPONSE, NAK};

  switch(serial_buffer[0]) {
    case SEND_JOB:
      response_buffer[2] = serial_buffer[3]; // Job id

      if(motor_ptr->status_variables.fault) {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;

            } else if (bytes_read == 8) {
                motor_ptr->command_variables.use_ramping = false;
                motor_ptr->command_variables.direction = (serial_buffer[1] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[2];
                motor_ptr->command_variables.job_id = serial_buffer[3];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[4]);
                motor_ptr->command_variables.pulse_interval = 0;
                motor_ptr->command_variables.pulse_on_period = 0;
                motor_ptr->command_variables.ramping_steps = 0;
                motor_ptr->StartJob();
                response_buffer[3] = 0x00;
                response_buffer[4] = ACK;

              } else {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                }
      break;

    case SEND_JOB_WITH_RAMPING:
      response_buffer[2] = serial_buffer[3]; // Job id

      if(motor_ptr->status_variables.fault) {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;
  
        } else if(motor_ptr->status_variables.running) {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;

            } else if (bytes_read == 12) {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[1] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[2];
                motor_ptr->command_variables.job_id = serial_buffer[3];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[4]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[8]);
                motor_ptr->command_variables.pulse_interval = 0;
                motor_ptr->command_variables.pulse_on_period = 0;
                motor_ptr->StartJob();
                response_buffer[3] = 0x00;
                response_buffer[4] = ACK;

              } else {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                }
      break;

    case SEND_JOB_ALL_VARIABLES:
      response_buffer[2] = serial_buffer[3]; // Job id

      if(false) { //motor_ptr->status_variables.fault) {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;

            } else if (bytes_read == 16) {
                motor_ptr->command_variables.use_ramping = false;
                motor_ptr->command_variables.direction = (serial_buffer[1] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[2];
                motor_ptr->command_variables.job_id = serial_buffer[3];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[4]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[8]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[12]);
                motor_ptr->StartJob();
                response_buffer[3] = 0x00;
                response_buffer[4] = ACK;
              } else {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                }
      break;

    case SEND_JOB_ALL_VARIABLES_WITH_RAMPING:
      response_buffer[2] = serial_buffer[3]; // Job id

      if(motor_ptr->status_variables.fault) {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;

            } else if (bytes_read == 20) {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[1] > 0) ? true : false;;
                motor_ptr->command_variables.microstep = serial_buffer[2];
                motor_ptr->command_variables.job_id = serial_buffer[3];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[4]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[8]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[12]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[16]);
                motor_ptr->StartJob();
                response_buffer[3] = 0x00;
                response_buffer[4] = ACK;                

              } else {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                }
      break;

    case SEND_JOB_ALL_VARIABLES_WITH_RAMPING_AND_RATE:
      response_buffer[2] = serial_buffer[3]; // Job id

      if(motor_ptr->status_variables.fault) {
        response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[3] = MOTOR_DISABLED_RESPONSE;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[3] = MOTOR_BUSY_RESPONSE;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[3] = MOTOR_IN_SLEEP_RESPONSE;

            } else if (bytes_read == 21) {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[1] > 0) ? true : false;;
                motor_ptr->command_variables.microstep = serial_buffer[2];
                motor_ptr->command_variables.job_id = serial_buffer[3];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[4]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[8]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[12]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[16]);
                motor_ptr->command_variables.ramp_scaler = serial_buffer[20];
                motor_ptr->StartJob();
                response_buffer[3] = 0x00;
                response_buffer[4] = ACK;                

              } else {
                  response_buffer[3] = BAD_JOB_COMMAND_RESPONSE;
                }
      break;

    case PAUSE_JOB:
        response_buffer[2] = serial_buffer[3]; // Job id

        if(motor_ptr->status_variables.fault) {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

          if(motor_ptr->status_variables.running) {
            motor_ptr->PauseJob();
          }

        } else if (motor_ptr->status_variables.enabled) {
            if(motor_ptr->status_variables.running) {
              if (motor_ptr->status_variables.paused) {
                response_buffer[3] = JOB_ALREADY_PAUSED_RESPONSE;

              } else {
                  motor_ptr->PauseJob();
                  response_buffer[3] = 0x00;
                  response_buffer[4] = ACK;                  
                }
            } else {
                response_buffer[3] = NO_ACTIVE_JOB_RESPONSE;

              }
          } else {
              response_buffer[3] = MOTOR_DISABLED_RESPONSE;
            }
      break;

    case RESUME_JOB:
        response_buffer[2] = serial_buffer[3]; // Job id

        if(motor_ptr->status_variables.fault) {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

        } else if (motor_ptr->status_variables.enabled) {
            if(motor_ptr->status_variables.running) {
              if(motor_ptr->status_variables.paused) {
                motor_ptr->ResumeJob();
                response_buffer[3] = 0x00;
                response_buffer[4] = ACK;
              } else {
                  response_buffer[3] = JOB_ALREADY_RESUMED_RESPONSE;
                }
            } else {
                response_buffer[3] = NO_ACTIVE_JOB_RESPONSE;
              }
          } else {
              response_buffer[3] = MOTOR_DISABLED_RESPONSE;
            }
      break;

    case CANCEL_JOB:
        response_buffer[2] = serial_buffer[3]; // Job id

        if (motor_ptr->status_variables.enabled) {
          if(motor_ptr->status_variables.running) {
            motor_ptr->CancelJob();

            uint8_t job_cancelled_buffer[JOB_CANCELLED_MESSAGE_LENGTH] = {JOB_CANCELLED_MESSAGE_ID, motor_ptr->status_variables.job_id};
            serialport.sendMessage(&job_cancelled_buffer[0], 2);

            motor_ptr->ResetJobId();
            response_buffer[3] = 0x00;
            response_buffer[4] = ACK;
          } else {
                response_buffer[3] = NO_ACTIVE_JOB_RESPONSE;
              }
          } else {
              response_buffer[3] = MOTOR_DISABLED_RESPONSE;
            }

      break;

    case ENABLE_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

        } else if (motor_ptr->status_variables.enabled) {
            response_buffer[3] = MOTOR_ALREADY_ENABLED_RESPONSE;

          } else {
              motor_ptr->Enable();
              response_buffer[3] = 0x00;
              response_buffer[4] = ACK;
            }

      break;

    case DISABLE_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

        } else if (motor_ptr->status_variables.enabled) {
            motor_ptr->Disable();
            response_buffer[3] = 0x00;
            response_buffer[4] = ACK;

          } else {
              response_buffer[3] = MOTOR_ALREADY_DISABLED_RESPONSE;
            }

      break;

    case SLEEP_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[4] = ACK;

          if(!motor_ptr->status_variables.sleep) {
            motor_ptr->Sleep();
          }

        } else if (motor_ptr->status_variables.sleep) {
            response_buffer[3] = MOTOR_ALREADY_SLEEPING_RESPONSE;

          } else {
              if(motor_ptr->status_variables.enabled && motor_ptr->status_variables.running) {
                motor_ptr->PauseJob();
                response_buffer[3] = SLEEP_WITH_ACTIVE_JOB_RESPONSE;
                response_buffer[4] = ACK;
              }

              motor_ptr->Sleep();
              response_buffer[3] = 0x00;
              response_buffer[4] = ACK;
            }

      break;

    case WAKE_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[3] = MOTOR_IN_FAULT_RESPONSE;

        } else if(motor_ptr->status_variables.sleep) {
            motor_ptr->Wake();
            if(motor_ptr->status_variables.enabled && motor_ptr->status_variables.running && motor_ptr->status_variables.paused) {
              response_buffer[3] = WAKE_WITH_ACTIVE_JOB_RESPONSE;
              response_buffer[4] = ACK;
              motor_ptr->ResumeJob();
            }

          } else {
              response_buffer[3] = MOTOR_ALREADY_AWAKE_RESPONSE;
            }

      break;

    case RESET_MOTOR:

      bool was_enabled = false;
      bool was_awake = false;
      bool was_running = false;

      if(motor_ptr->status_variables.enabled) {
        was_enabled = true;

        if(motor_ptr->status_variables.running) {
          motor_ptr->PauseJob();
          was_running = true;
        }

        if(!motor_ptr->status_variables.sleep) {
          motor_ptr->Sleep();
          was_awake = true;
        }
      }

      motor_ptr->Reset();

      if(was_enabled) {
        motor_ptr->Enable();
      }

      if(was_awake) {
        motor_ptr->Wake();
      }

      if(was_running) {
        motor_ptr->ResumeJob();
      }

      response_buffer[3] = 0x00;
      response_buffer[4] = ACK;
      
      break;
  }

  serialport.sendMessage(&response_buffer[0], RESPONSE_MESSAGE_LENGTH);
}
