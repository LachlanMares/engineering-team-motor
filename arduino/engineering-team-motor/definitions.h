// This file is for Arduino and Python Interface be careful when adding stuff
// "// some_text" will be a python dictionary key until next "// other_text". 
// Do not put comments at the end of #define lines

// serial_settings
#define BAUD_RATE                           1000000
#define SERIAL_BUFFER_LENGTH                256
#define SERIAL_TIMEOUT_MS                   100
#define STX                                 0x02
#define ETX                                 0x03
#define ACK                                 0x06
#define NAK                                 0x15

// encoder_settings
#define ENCODER_UPDATE_PERIOD_US  25000
#define ENCODER_PULSES_PER_REVOLUTION  2400

// schedule_settings
#define FAULT_CHECK_TASK_ID  0
#define FAULT_CHECK_INTERVAL_US  100000
#define STATUS_MESSAGE_TASK_ID  1
#define STATUS_MESSAGE_INTERVAL_US  250000
#define MOTOR_FEEDBACK_TASK_ID  2
#define MOTOR_FEEDBACK_INTERVAL_US  10000

// io_settings
#define MOTOR_INT_PIN_A 2
#define MOTOR_INT_PIN_B 3
#define MOTOR_INT_PIN_Z 4
#define DIRECTION_PIN 5
#define STEP_PIN 6
#define SLEEP_PIN 7
#define RESET_PIN 8
#define FAULT_PIN 9
#define M0_PIN 14
#define M1_PIN 15
#define M2_PIN 16
#define ENABLE_PIN 10

// status_message_bits
#define STATUS_DIRECTION_BIT                0
#define STATUS_FAULT_BIT                    1
#define STATUS_PAUSED_BIT                   2
#define STATUS_RAMPING_BIT                  3
#define STATUS_SPARE_BIT                    4
#define STATUS_ENABLED_BIT                  5
#define STATUS_RUNNING_BIT                  6
#define STATUS_SLEEP_BIT                    7

// message_types
#define MOTOR_STATUS_MESSAGE_ID             0xFF
#define MOTOR_FEEDBACK_MESSAGE_ID           0xFE
#define MOTOR_FAULT_MESSAGE_ID              0xFD
#define RESPONSE_MESSAGE_ID                 0xFC
#define JOB_COMPLETE_MESSAGE_ID             0xFA
#define JOB_CANCELLED_MESSAGE_ID            0xF9

// message_lengths
#define MOTOR_STATUS_MESSAGE_LENGTH         8
#define MOTOR_FEEDBACK_MESSAGE_LENGTH       11
#define MOTOR_FAULT_MESSAGE_LENGTH          1
#define RESPONSE_MESSAGE_LENGTH             5
#define JOB_COMPLETE_MESSAGE_LENGTH         2
#define JOB_CANCELLED_MESSAGE_LENGTH        2

// command_types
#define SEND_JOB                            0xEF
#define SEND_JOB_WITH_RAMPING               0xEE
#define SEND_JOB_ALL_VARIABLES              0xED
#define SEND_JOB_ALL_VARIABLES_WITH_RAMPING 0xEC
#define PAUSE_JOB                           0xEB
#define RESUME_JOB                          0xEA
#define CANCEL_JOB                          0xE9
#define ENABLE_MOTOR                        0xE8
#define DISABLE_MOTOR                       0xE7
#define SLEEP_MOTOR                         0xE6
#define WAKE_MOTOR                          0xE5
#define RESET_MOTOR                         0xE4

// response_types
#define BAD_JOB_COMMAND_RESPONSE            0xDF
#define MOTOR_BUSY_RESPONSE                 0xDE
#define UNKNOWN_MOTOR_COMMAND_RESPONSE      0xDD
#define MOTOR_IN_FAULT_RESPONSE             0xDC
#define MOTOR_IN_SLEEP_RESPONSE             0xDB
#define MOTOR_PAUSED_RESPONSE               0xDA
#define MOTOR_DISABLED_RESPONSE             0xD9
#define NO_ACTIVE_JOB_RESPONSE              0xD8
#define JOB_ALREADY_PAUSED_RESPONSE         0xD7
#define JOB_ALREADY_RESUMED_RESPONSE        0xD6
#define MOTOR_ALREADY_ENABLED_RESPONSE      0xD5
#define MOTOR_ALREADY_DISABLED_RESPONSE     0xD4
#define MOTOR_ALREADY_SLEEPING_RESPONSE     0xD3
#define MOTOR_ALREADY_AWAKE_RESPONSE        0xD2
#define SLEEP_WITH_ACTIVE_JOB_RESPONSE      0xD1
#define WAKE_WITH_ACTIVE_JOB_RESPONSE       0xD0

// motor_settings
#define MOTOR_ID                            0x00
#define MOTOR_STEPS_PER_REV                 200
#define DEFAULT_PULSE_ON_PERIOD             500
#define DEFAULT_PULSE_INTERVAL              2000
#define DEFAULT_RAMP_STEPS                  50
#define MINIMUM_PULSE_INTERVAL              1000
#define MAXIMUM_PULSE_INTERVAL              1000000