#include "MotorInterface.h"

MotorInterface::MotorInterface(unsigned long update_period_us, int ppr, bool filter) : QuadratureEncoder(update_period_us, ppr, filter) {
    _output_state = false;

    status_variables.direction = false;
    status_variables.running = false;
    status_variables.fault = false;
    status_variables.enabled = false;
    status_variables.paused = false;
    status_variables.sleep = false;
    status_variables.use_ramping = false;
    status_variables.running = false;
    status_variables.microstep = 1;
    status_variables.job_id = 0;
    status_variables.ramp_up_stop = 0;
    status_variables.ramp_down_start = 0;
    status_variables.pulses_remaining = 0;
    status_variables.pulse_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.ramp_pulse_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.pulse_on_period = DEFAULT_PULSE_ON_PERIOD;
    status_variables.ramp_up_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.ramp_down_interval = DEFAULT_PULSE_INTERVAL;
    status_variables.ramp_interval_step = 0;

    command_variables.direction = false;
    command_variables.use_ramping = false;
    command_variables.microstep = 1;
    command_variables.job_id = 0;
    command_variables.ramping_steps = 0;
    command_variables.pulses = 0;
    command_variables.pulse_interval = DEFAULT_PULSE_INTERVAL;
    command_variables.pulse_on_period = DEFAULT_PULSE_ON_PERIOD;

    encoder.direction = false;
    encoder.angle_count = 0;
    encoder.velocity_radians = 0.0;
    encoder.angle_radians = 0.0;
    encoder.count = 0;
}

void MotorInterface::Init(int direction_pin, int step_pin, int sleep_pin, int reset_pin, int fault_pin, int m0_pin, int m1_pin, int m2_pin, int enable_pin) {
    _direction_pin = direction_pin;
    _step_pin = step_pin;
    _sleep_pin = sleep_pin;
    _reset_pin = reset_pin;
    _fault_pin = fault_pin;
    _m0_pin = m0_pin;
    _m1_pin = m1_pin; 
    _m2_pin = m2_pin;
    _enable_pin = enable_pin;

    pinMode(_direction_pin, OUTPUT);
    pinMode(_step_pin, OUTPUT);
    pinMode(_sleep_pin, OUTPUT);
    pinMode(_reset_pin, OUTPUT);
    pinMode(_fault_pin, INPUT_PULLUP);
    pinMode(_m0_pin, OUTPUT);
    pinMode(_m1_pin, OUTPUT);
    pinMode(_m2_pin, OUTPUT);
    pinMode(_enable_pin, OUTPUT);

    digitalWrite(_direction_pin, LOW);
    digitalWrite(_step_pin, LOW);
    digitalWrite(_sleep_pin, HIGH);
    digitalWrite(_reset_pin, HIGH);
    digitalWrite(_m0_pin, LOW);
    digitalWrite(_m1_pin, LOW);
    digitalWrite(_m2_pin, LOW);
    digitalWrite(_enable_pin, LOW);
} 

void MotorInterface::Enable() {
    status_variables.enabled = true;
    digitalWrite(_enable_pin, LOW);
}

void MotorInterface::Disable() {
    status_variables.enabled = false;
    digitalWrite(_enable_pin, HIGH);
}

void MotorInterface::Wake() {
    status_variables.sleep = false;
    digitalWrite(_sleep_pin, HIGH);
}

void MotorInterface::Sleep() {
    status_variables.sleep = true;
    digitalWrite(_sleep_pin, LOW);
}

void MotorInterface::Reset() {
    digitalWrite(_reset_pin, LOW);
    delay(1);
    digitalWrite(_reset_pin, HIGH);
}

boolean MotorInterface::FaultStatus() {
    return false; // !digitalRead(_fault_pin);
}

void MotorInterface::DecodeMicroStep() {
    switch(status_variables.microstep)
    {
        case 1:
            digitalWrite(_m0_pin, LOW);
            digitalWrite(_m1_pin, LOW);
            digitalWrite(_m2_pin, LOW);
            break;

        case 2:
            digitalWrite(_m0_pin, HIGH);
            digitalWrite(_m1_pin, LOW);
            digitalWrite(_m2_pin, LOW);
            break;

        case 4:
            digitalWrite(_m0_pin, LOW);
            digitalWrite(_m1_pin, HIGH);
            digitalWrite(_m2_pin, LOW);
            break;

        case 8:
            digitalWrite(_m0_pin, HIGH);
            digitalWrite(_m1_pin, HIGH);
            digitalWrite(_m2_pin, LOW);
            break;

        case 16:
            digitalWrite(_m0_pin, LOW);
            digitalWrite(_m1_pin, LOW);
            digitalWrite(_m2_pin, HIGH);
            break;

        case 32:
            digitalWrite(_m0_pin, HIGH);
            digitalWrite(_m1_pin, LOW);
            digitalWrite(_m2_pin, HIGH);
            break;

        default:
            digitalWrite(_m0_pin, LOW);
            digitalWrite(_m1_pin, LOW);
            digitalWrite(_m2_pin, LOW);
            status_variables.microstep = 1;
            break;
    }
}

void MotorInterface::StartJob() {
    status_variables.fault = FaultStatus();

    if(status_variables.fault) {
        status_variables.running = false;
        Sleep();
        Disable();

    } else {

        status_variables.running = true;
        status_variables.direction = command_variables.direction;
        status_variables.use_ramping = command_variables.use_ramping;
        status_variables.microstep = command_variables.microstep;
        status_variables.job_id = command_variables.job_id;
        status_variables.paused = false;

        _output_state = false;
        _last_pulse_on_micros = 0;
        _last_pulse_off_micros = 0;

        if (status_variables.sleep) {
            Wake();
            Reset();
        }

        if (!status_variables.enabled) {
            Enable();
        }

        digitalWrite(_direction_pin, status_variables.direction ? HIGH : LOW);
        digitalWrite(_step_pin, LOW);
        DecodeMicroStep();

        status_variables.pulse_interval = (command_variables.pulse_interval > MINIMUM_PULSE_INTERVAL && command_variables.pulse_interval < MAXIMUM_PULSE_INTERVAL) ? command_variables.pulse_interval : DEFAULT_PULSE_INTERVAL;
        status_variables.pulse_on_period = (command_variables.pulse_on_period < command_variables.pulse_interval && command_variables.pulse_on_period != 0) ? command_variables.pulse_on_period : (long)(status_variables.pulse_interval/2);
        status_variables.pulses_remaining = command_variables.pulses;

        if(status_variables.use_ramping) {
            if (command_variables.ramping_steps == 0) {
                command_variables.ramping_steps = DEFAULT_RAMP_STEPS;
            }

            if(2 * command_variables.ramping_steps < status_variables.pulses_remaining) {
                // There are enough steps remaining to use specified ramp 
                status_variables.ramp_up_stop = status_variables.pulses_remaining - command_variables.ramping_steps;
                status_variables.ramp_down_start = command_variables.ramping_steps;

            } else {
                status_variables.ramp_up_stop = (long)(status_variables.pulses_remaining / 2);
                status_variables.ramp_down_start = status_variables.ramp_up_stop - 1;
            }

            status_variables.ramp_up_interval = status_variables.pulse_interval * 3;
            status_variables.ramp_pulse_interval = status_variables.ramp_up_interval;
            status_variables.ramp_down_interval = status_variables.pulse_interval;
            status_variables.ramp_interval_step = (long)((status_variables.ramp_up_interval - status_variables.ramp_down_interval) / command_variables.ramping_steps);

        } else {
            status_variables.ramp_up_stop = 0;
            status_variables.ramp_down_start = 0;
            status_variables.ramp_up_interval = 0;
            status_variables.ramp_down_interval = 0;
            status_variables.ramp_interval_step = 0;
            status_variables.ramp_pulse_interval = 0;
        }
    }
}

void MotorInterface::PauseJob() {
    status_variables.paused = true;
}

void MotorInterface::ResumeJob() {
    status_variables.paused = false;
    _last_pulse_on_micros = 0;
    _last_pulse_off_micros = 0;
}

void MotorInterface::CancelJob() {
    status_variables.running = false;
    _output_state = false;
    status_variables.pulses_remaining = 0;
    digitalWrite(_step_pin, LOW);
}

void MotorInterface::ResetJobId() {
    status_variables.job_id = 0;
}

bool MotorInterface::Update(unsigned long micros_now) {
    if (updateEncoderVelocity(micros_now)) {
        /*
        encoder.direction = getEncoderDirection();
        encoder.count = getEncoderCount();
        encoder.angle_count = getEncoderAngleCount();
        encoder.angle_radians = getEncoderAngleRadians();
        encoder.velocity_radians = getEncoderVelocityRadians();
        */
    }

    bool job_done = false;

    if(status_variables.enabled) {
        
        if(status_variables.running && !status_variables.paused && !status_variables.fault) {

            if(status_variables.pulses_remaining > 0) {

                if(status_variables.use_ramping) {
                    if(!_output_state) {

                        if(abs(micros_now - _last_pulse_on_micros) >= status_variables.ramp_pulse_interval) {

                            _last_pulse_on_micros += status_variables.pulse_interval;
                            _last_pulse_off_micros = micros_now;
                            _output_state = true;
                            digitalWrite(_step_pin, HIGH);

                            if(status_variables.pulses_remaining > status_variables.ramp_up_stop) {
                                status_variables.ramp_up_interval -= status_variables.ramp_interval_step;
                                status_variables.ramp_pulse_interval = status_variables.ramp_up_interval;

                            } else if(status_variables.pulses_remaining < status_variables.ramp_down_start) {
                                status_variables.ramp_down_interval += status_variables.ramp_interval_step;
                                status_variables.ramp_pulse_interval = status_variables.ramp_down_interval;

                            } else {
                                status_variables.ramp_pulse_interval = status_variables.pulse_interval;
                            }
                        }

                    } else {
                            if(abs(micros_now - _last_pulse_off_micros) >= status_variables.pulse_on_period) {
                                digitalWrite(_step_pin, LOW);
                                _output_state = false;
                                status_variables.pulses_remaining--;
                            }
                        }

                } else {
                        if(!_output_state) {
                            if(abs(micros_now - _last_pulse_on_micros) >= status_variables.pulse_interval) {
                                digitalWrite(_step_pin, HIGH);
                                _last_pulse_on_micros = micros_now;
                                _last_pulse_off_micros = micros_now;
                                _output_state = true;
                            }

                        } else {
                            if(abs(micros_now - _last_pulse_off_micros) >= status_variables.pulse_on_period) {
                                    digitalWrite(_step_pin, LOW);
                                    _output_state = false;
                                    status_variables.pulses_remaining--;
                                }
                        }
                }

            } else {
                status_variables.running = false;
                job_done = true;
            }
        }
    }

    return job_done;
}

bool MotorInterface::FaultCheck() {
    status_variables.fault = FaultStatus();
    return status_variables.fault;
}

void MotorInterface::UpdateStatus() {
    bitWrite(status_byte, DIRECTION_BIT, status_variables.direction); // Bit 0
    bitWrite(status_byte, FAULT_BIT, status_variables.fault); // Bit 1
    bitWrite(status_byte, PAUSED_BIT, status_variables.paused); // Bit 2
    bitWrite(status_byte, USE_RAMPING_BIT, status_variables.use_ramping); // Bit 3
    // bitWrite(status_byte, 4, ); // Bit 4
    bitWrite(status_byte, ENABLE_BIT, status_variables.enabled); // Bit 5
    bitWrite(status_byte, RUNNING_BIT, status_variables.running); // Bit 6
    bitWrite(status_byte, SLEEP_BIT, status_variables.sleep); // Bit 7
}


