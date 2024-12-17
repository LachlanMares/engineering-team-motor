#ifndef MotorInterface_h
#define MotorInterface_h

#include "Arduino.h"
#include "QuadratureEncoder.h"

// Status bits
#define DIRECTION_BIT 0
#define FAULT_BIT 1
#define PAUSED_BIT 2
#define USE_RAMPING_BIT 3
// #define SPARE_BIT 4
#define ENABLE_BIT 5
#define RUNNING_BIT 6
#define SLEEP_BIT 7

#define MINIMUM_PULSE_INTERVAL_US           500
#define DEFAULT_PULSE_ON_PERIOD             500
#define DEFAULT_PULSE_INTERVAL              1000
#define DEFAULT_RAMP_STEPS                  50
#define MINIMUM_PULSE_INTERVAL              500
#define MAXIMUM_PULSE_INTERVAL              1000000

#define MAF_FILTER_LENGTH  10

struct motor_command_struct {
  bool direction;
  bool use_ramping;
  uint8_t microstep;
  uint8_t job_id;
  unsigned long ramping_steps;
  unsigned long pulse_interval;
  unsigned long pulses;
  unsigned long pulse_on_period;
};

struct motor_status_struct {
  bool running;
  bool fault;
  bool direction;
  bool enabled;
  bool sleep;
  bool paused;
  bool use_ramping;
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
  bool direction;
  int angle_count;
  float velocity_radians;
  float angle_radians;
  long count;
};

class MotorInterface : public QuadratureEncoder {
  public:
    MotorInterface(unsigned long update_period_us, int ppr);
    void Init(int direction_pin, int step_pin, int sleep_pin, int reset_pin, int fault_pin, int m0_pin, int m1_pin, int m2_pin, int enable_pin);
    void Enable();
    void Disable();
    void Wake();
    void Sleep();
    void Reset();
    bool FaultStatus();
    void StartJob();
    void PauseJob();
    void ResumeJob();
    void CancelJob();
    void ResetJobId();
    bool FaultCheck();
    void UpdateStatus();
    bool Update(unsigned long);

    motor_command_struct command_variables;
    motor_status_struct status_variables;
    encoder_status_struct encoder;
    uint8_t status_byte;

  private:
    void DecodeMicroStep();

    bool _output_state;
    int _direction_pin, _step_pin, _sleep_pin, _reset_pin, _fault_pin, _m0_pin, _m1_pin, _m2_pin, _enable_pin;
    float filter_buffer[MAF_FILTER_LENGTH];
    unsigned long _last_pulse_on_micros, _last_pulse_off_micros; 
};

#endif
