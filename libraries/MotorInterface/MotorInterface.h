#ifndef MotorInterface_h
#define MotorInterface_h

#include "Arduino.h"
#include "QuadratureEncoder.h"

// IO Pins 
#define DIRECTION_PIN 5
#define STEP_PIN 6
#define SLEEP_PIN 7
#define RESET_PIN 8
#define FAULT_PIN 9
#define M0_PIN 10
#define M1_PIN 11
#define M2_PIN 12
#define ENABLE_PIN 13

// Status bits
#define DIRECTION_BIT 0
#define FAULT_BIT 1
#define PAUSED_BIT 2
#define USE_RAMPING_BIT 3
// #define SPARE_BIT 4
#define ENABLE_BIT 5
#define RUNNING_BIT 6
#define SLEEP_BIT 7

#define DEFAULT_PULSE_ON_PERIOD 500
#define DEFAULT_PULSE_INTERVAL  1000
#define DEFAULT_RAMP_STEPS      50
#define MINIMUM_PULSE_INTERVAL  500
#define MAXIMUM_PULSE_INTERVAL  1000000
#define FAULT_CHECK_INTERVAL    100000
#define STATUS_INTERVAL         10000

#define MAF_FILTER_LENGTH  10

struct motor_command_struct {
  boolean direction;
  boolean use_ramping;
  uint8_t microstep;
  uint8_t job_id;
  unsigned long ramping_steps;
  unsigned long pulse_interval;
  unsigned long pulses;
  unsigned long pulse_on_period;
};

struct motor_status_struct {
  boolean running;
  boolean fault;
  boolean direction;
  boolean enabled;
  boolean sleep;
  boolean paused;
  boolean use_ramping;
  uint8_t microstep;
  uint8_t job_id;
  unsigned long ramp_up_stop;
  unsigned long ramp_down_start;
  unsigned long ramp_up_interval;
  unsigned long ramp_down_interval;
  unsigned long ramp_interval_step;
  unsigned long ramp_pulse_interval;
  unsigned long pulse_interval;
  unsigned long pulse_on_period;
  unsigned long pulses_remaining;
};

struct encoder_status_struct {
  boolean direction;
  int angle_count;
  float velocity_radians;
  float angle_radians;
  long count;
};

class MotorInterface : public QuadratureEncoder {
  public:
    MotorInterface(unsigned long update_period_us, int ppr);
    void Enable();
    void Disable();
    void Wake();
    void Sleep();
    void Reset();
    boolean FaultStatus();
    void StartJob();
    void PauseJob();
    void ResumeJob();
    void CancelJob();
    void ResetJobId();
    boolean Update(unsigned long);

    motor_command_struct command_variables;
    motor_status_struct status_variables;
    encoder_status_struct encoder;
    uint8_t status_byte;

  private:
    void DecodeMicroStep();

    boolean _output_state;
    float filter_buffer[MAF_FILTER_LENGTH];
    unsigned long _last_fault_check_micros, _fault_check_interval, _status_interval, _last_status_micros;
    unsigned long _last_pulse_on_micros, _last_pulse_off_micros, _last_micros;
};

#endif
