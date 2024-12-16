void initialiseSerial() {
  serialport.setInitial(BAUD_RATE, SERIAL_TIMEOUT_MS);
}

void updateSerial() {
  int bytes_read = serialport.update(&serial_buffer[0]);

  if(bytes_read > 0) {
    switch(serial_buffer[0]) {
      case MOTOR_ID:
        processCommandMessage(&motor, MOTOR_ID, bytes_read);
        break;

      default:
        break;
    }
  }

  if (scheduler.taskReady(STATUS_MESSAGE_TASK_ID)) {
    uint8_t motor_status_buffer[8];
    motor_status_buffer[0] = MOTOR_STATUS_MESSAGE_ID;
    motor_status_buffer[1] = motor.status_byte;
    motor_status_buffer[2] = motor.status_variables.job_id; 
    motor_status_buffer[3] = motor.status_variables.microstep;
    bytesFromUnsignedLong(motor.status_variables.pulses_remaining, &motor_status_buffer[3]);
    serialport.sendMessage(&motor_status_buffer[0], 8);
  }

  if (scheduler.taskReady(MOTOR_FEEDBACK_TASK_ID)) {
    uint8_t motor_feedback_buffer[13];
    motor_feedback_buffer[0] = MOTOR_FEEDBACK_MESSAGE_ID;
    bytesFromFloat32Bit(motor.encoder.velocity_radians, &motor_feedback_buffer[1], 1000);
    bytesFromFloat32Bit(motor.encoder.angle_radians, &motor_feedback_buffer[7], 1000);
    serialport.sendMessage(&motor_feedback_buffer[0], 13);
  }
}

void processCommandMessage(MotorInterface* motor_ptr, uint8_t motor_num, int bytes_read) {
  uint8_t response_buffer[4] = {RESPONSE_MESSAGE_ID, serial_buffer[1], 0x00, ACK};

  switch(serial_buffer[1]) {

    case SEND_JOB:
      if(motor_ptr->status_variables.fault) {
        response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[3] = NAK;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[2] = MOTOR_DISABLED_RESPONSE;
          response_buffer[3] = NAK;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[2] = MOTOR_BUSY_RESPONSE;
            response_buffer[3] = NAK;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[2] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[3] = NAK;

            } else if (bytes_read == 9) {
                motor_ptr->command_variables.use_ramping = false;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.pulse_interval = 0;
                motor_ptr->command_variables.pulse_on_period = 0;
                motor_ptr->command_variables.ramping_steps = 0;
                motor_ptr->StartJob();

              } else {
                  response_buffer[2] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[3] = NAK;
                }
      break;

    case SEND_JOB_WITH_RAMPING:
      if(motor_ptr->status_variables.fault) {
        response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[3] = NAK;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[2] = MOTOR_DISABLED_RESPONSE;
          response_buffer[3] = NAK;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[2] = MOTOR_BUSY_RESPONSE;
            response_buffer[3] = NAK;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[2] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[3] = NAK;

            } else if (bytes_read == 13) {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[9]);
                motor_ptr->command_variables.pulse_interval = 0;
                motor_ptr->command_variables.pulse_on_period = 0;
                motor_ptr->StartJob();

              } else {
                  response_buffer[2] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[3] = NAK;
                }
      break;

    case SEND_JOB_ALL_VARIABLES:

      if(motor_ptr->status_variables.fault) {
        response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[3] = NAK;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[2] = MOTOR_DISABLED_RESPONSE;
          response_buffer[3] = NAK;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[2] = MOTOR_BUSY_RESPONSE;
            response_buffer[3] = NAK;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[2] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[3] = NAK;

            } else if (bytes_read == 17) {
                motor_ptr->command_variables.use_ramping = false;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[9]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[13]);
                motor_ptr->StartJob();

              } else {
                  response_buffer[2] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[3] = NAK;
                }
      break;

    case SEND_JOB_ALL_VARIABLES_WITH_RAMPING:

      if(motor_ptr->status_variables.fault) {
        response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
        response_buffer[3] = NAK;

      } else if (!motor_ptr->status_variables.enabled) {
          response_buffer[2] = MOTOR_DISABLED_RESPONSE;
          response_buffer[3] = NAK;

        } else if(motor_ptr->status_variables.running) {
            response_buffer[2] = MOTOR_BUSY_RESPONSE;
            response_buffer[3] = NAK;

          } else if (motor_ptr->status_variables.sleep) {
              response_buffer[2] = MOTOR_IN_SLEEP_RESPONSE;
              response_buffer[3] = NAK;

            } else if (bytes_read == 21) {
                motor_ptr->command_variables.use_ramping = true;
                motor_ptr->command_variables.direction = (serial_buffer[2] > 0) ? true : false;;
                motor_ptr->command_variables.microstep = serial_buffer[3];
                motor_ptr->command_variables.job_id = serial_buffer[4];
                motor_ptr->command_variables.pulses = longFromBytes(&serial_buffer[5]);
                motor_ptr->command_variables.pulse_interval = longFromBytes(&serial_buffer[9]);
                motor_ptr->command_variables.pulse_on_period = longFromBytes(&serial_buffer[13]);
                motor_ptr->command_variables.ramping_steps = longFromBytes(&serial_buffer[17]);
                motor_ptr->StartJob();

              } else {
                  response_buffer[2] = BAD_JOB_COMMAND_RESPONSE;
                  response_buffer[3] = NAK;
                }
      break;

    case PAUSE_JOB:

        if(motor_ptr->status_variables.fault) {
          response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;

          if(motor_ptr->status_variables.running) {
            motor_ptr->PauseJob();
          }

        } else if (motor_ptr->status_variables.enabled) {
            if(motor_ptr->status_variables.running) {
              if (motor_ptr->status_variables.paused) {
                response_buffer[2] = JOB_ALREADY_PAUSED_RESPONSE;
                response_buffer[3] = NAK;

              } else {
                  motor_ptr->PauseJob();
                }
            } else {
                response_buffer[2] = NO_ACTIVE_JOB_RESPONSE;
                response_buffer[3] = NAK;
              }
          } else {
              response_buffer[2] = MOTOR_DISABLED_RESPONSE;
              response_buffer[3] = NAK;
            }
      break;

    case RESUME_JOB:

        if(motor_ptr->status_variables.fault) {
          response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[3] = NAK;

        } else if (motor_ptr->status_variables.enabled) {
            if(motor_ptr->status_variables.running) {
              if(motor_ptr->status_variables.paused) {
                motor_ptr->ResumeJob();

              } else {
                  response_buffer[2] = JOB_ALREADY_RESUMED_RESPONSE;
                  response_buffer[3] = NAK;
                }
            } else {
                response_buffer[2] = NO_ACTIVE_JOB_RESPONSE;
                response_buffer[3] = NAK;
              }
          } else {
              response_buffer[2] = MOTOR_DISABLED_RESPONSE;
              response_buffer[3] = NAK;
            }

      break;

    case CANCEL_JOB:

        if (motor_ptr->status_variables.enabled) {
          if(motor_ptr->status_variables.running) {
            motor_ptr->CancelJob();
            uint8_t job_cancelled_buffer[3] = {JOB_CANCELLED_MESSAGE_ID, motor_num, motor_ptr->status_variables.job_id};
            serialport.sendMessage(&job_cancelled_buffer[0], 3);
            motor_ptr->ResetJobId();

          } else {
                response_buffer[2] = NO_ACTIVE_JOB_RESPONSE;
                response_buffer[3] = NAK;
              }
          } else {
              response_buffer[2] = MOTOR_DISABLED_RESPONSE;
              response_buffer[3] = NAK;
            }

      break;

    case ENABLE_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[3] = NAK;

        } else if (motor_ptr->status_variables.enabled) {
            response_buffer[2] = MOTOR_ALREADY_ENABLED_RESPONSE;
            response_buffer[3] = NAK;

          } else {
              motor_ptr->Enable();
            }

      break;

    case DISABLE_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[3] = NAK;

        } else if (motor_ptr->status_variables.enabled) {
            motor_ptr->Disable();

          } else {
              response_buffer[2] = MOTOR_ALREADY_DISABLED_RESPONSE;
              response_buffer[3] = NAK;
            }

      break;

    case SLEEP_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
          if(!motor_ptr->status_variables.sleep) {
            motor_ptr->Sleep();
          }

        } else if (motor_ptr->status_variables.sleep) {
            response_buffer[2] = MOTOR_ALREADY_SLEEPING_RESPONSE;
            response_buffer[3] = NAK;

          } else {
              if(motor_ptr->status_variables.enabled && motor_ptr->status_variables.running) {
                motor_ptr->PauseJob();
                response_buffer[2] = SLEEP_WITH_ACTIVE_JOB_RESPONSE;
              }
              motor_ptr->Sleep();
            }

      break;

    case WAKE_MOTOR:

        if(motor_ptr->status_variables.fault) {
          response_buffer[2] = MOTOR_IN_FAULT_RESPONSE;
          response_buffer[3] = NAK;

        } else if(motor_ptr->status_variables.sleep) {
            motor_ptr->Wake();
            if(motor_ptr->status_variables.enabled && motor_ptr->status_variables.running && motor_ptr->status_variables.paused) {
              response_buffer[2] = WAKE_WITH_ACTIVE_JOB_RESPONSE;
              motor_ptr->ResumeJob();
            }

          } else {
              response_buffer[2] = MOTOR_ALREADY_AWAKE_RESPONSE;
              response_buffer[3] = NAK;
            }

      break;

    case RESET_MOTOR:

      boolean was_enabled = false;
      boolean was_awake = false;
      boolean was_running = false;

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

      break;

    default:
      response_buffer[2] = UNKNOWN_MOTOR_COMMAND_RESPONSE;
      response_buffer[3] = NAK;
      break;
  }

  serialport.sendMessage(&response_buffer[0], 4);
}
