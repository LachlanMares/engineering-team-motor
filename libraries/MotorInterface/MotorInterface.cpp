#include "MotorInterface.h"

MotorInterface::MotorInterface(unsigned long update_period_us) : QuadratureEncoder(update_period_us)
{
    _output_state = false;
    _fault_check_interval = FAULT_CHECK_INTERVAL;
    _status_interval = STATUS_INTERVAL;
    _last_fault_check_micros = micros();
    _last_status_micros = micros();

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
    encoder.velocity = 0.0;
    encoder.count = 0;

    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);
    pinMode(FAULT_PIN, INPUT_PULLUP);
    pinMode(M0_PIN, OUTPUT);
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(DIRECTION_PIN, LOW);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(RESET_PIN, HIGH);
    digitalWrite(M0_PIN, LOW);
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);
    digitalWrite(ENABLE_PIN, LOW);
}

void MotorInterface::Enable() {
    status_variables.enabled = true;
    digitalWrite(ENABLE_PIN, LOW);
}

void MotorInterface::Disable() {
    status_variables.enabled = false;
    digitalWrite(ENABLE_PIN, HIGH);
}

void MotorInterface::Wake() {
    status_variables.sleep = false;
    digitalWrite(SLEEP_PIN, HIGH);
}

void MotorInterface::Sleep() {
    status_variables.sleep = true;
    digitalWrite(SLEEP_PIN, LOW);
}

void MotorInterface::Reset() {
    digitalWrite(RESET_PIN, LOW);
    delay(1);
    digitalWrite(RESET_PIN, HIGH);
}

boolean MotorInterface::FaultStatus() {
    return !digitalRead(FAULT_PIN);
}

void MotorInterface::DecodeMicroStep() {
    switch(status_variables.microstep)
    {
        case 1:
            digitalWrite(M0_PIN, LOW);
            digitalWrite(M1_PIN, LOW);
            digitalWrite(M2_PIN, LOW);
            break;

        case 2:
            digitalWrite(M0_PIN, HIGH);
            digitalWrite(M1_PIN, LOW);
            digitalWrite(M2_PIN, LOW);
            break;

        case 4:
            digitalWrite(M0_PIN, LOW);
            digitalWrite(M1_PIN, HIGH);
            digitalWrite(M2_PIN, LOW);
            break;

        case 8:
            digitalWrite(M0_PIN, HIGH);
            digitalWrite(M1_PIN, HIGH);
            digitalWrite(M2_PIN, LOW);
            break;

        case 16:
            digitalWrite(M0_PIN, LOW);
            digitalWrite(M1_PIN, LOW);
            digitalWrite(M2_PIN, HIGH);
            break;

        case 32:
            digitalWrite(M0_PIN, HIGH);
            digitalWrite(M1_PIN, LOW);
            digitalWrite(M2_PIN, HIGH);
            break;

        default:
            digitalWrite(M0_PIN, LOW);
            digitalWrite(M1_PIN, LOW);
            digitalWrite(M2_PIN, LOW);
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
        _last_micros = 0;
        _last_pulse_on_micros = 0;
        _last_pulse_off_micros = 0;

        digitalWrite(SLEEP_PIN, HIGH);
        digitalWrite(RESET_PIN, HIGH);
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(DIRECTION_PIN, status_variables.direction ? HIGH : LOW);
        digitalWrite(STEP_PIN, LOW);
        DecodeMicroStep();

        status_variables.pulse_interval = (command_variables.pulse_interval > MINIMUM_PULSE_INTERVAL && command_variables.pulse_interval < MAXIMUM_PULSE_INTERVAL) ? command_variables.pulse_interval : DEFAULT_PULSE_INTERVAL;
        status_variables.pulse_on_period = (command_variables.pulse_on_period < command_variables.pulse_interval) ? command_variables.pulse_on_period : (long)(command_variables.pulse_interval/2);
        status_variables.pulses_remaining = command_variables.pulses;

        if(status_variables.use_ramping) {
            if(2 * command_variables.ramping_steps < status_variables.pulses_remaining) {
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

        } else
            {
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
    digitalWrite(STEP_PIN, LOW);
}

void MotorInterface::ResetJobId() {
    status_variables.job_id = 0;
}

boolean MotorInterface::Update(unsigned long micros_now) {
    
    if (updateEncoder(micros_now)) {
        encoder.direction = getEncoderDirection();
        encoder.velocity = getEncoderVelocity();
        encoder.count = getEncoderCount();
    }

    boolean job_done = false;

    if(status_variables.enabled) {
        if(status_variables.running && !status_variables.paused && !status_variables.fault) {

            if(status_variables.pulses_remaining > 0) {

                if(status_variables.use_ramping) {

                    if(!_output_state) {

                        if(abs(micros_now - _last_pulse_on_micros) >= status_variables.ramp_pulse_interval) {

                            _last_pulse_on_micros += status_variables.pulse_interval;
                            _last_pulse_off_micros = micros_now;
                            _output_state = true;
                            digitalWrite(STEP_PIN, HIGH);

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
                                digitalWrite(STEP_PIN, LOW);
                                _output_state = false;
                                status_variables.pulses_remaining--;
                            }
                        }

                } else {
                        if(!_output_state) {
                            if(abs(micros_now - _last_pulse_on_micros) >= status_variables.pulse_interval) {
                                digitalWrite(STEP_PIN, HIGH);
                                _last_pulse_on_micros = micros_now;
                                _last_pulse_off_micros = micros_now;
                                _output_state = true;
                            }

                        } else {
                            if(abs(micros_now - _last_pulse_off_micros) >= status_variables.pulse_on_period) {
                                    digitalWrite(STEP_PIN, LOW);
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

    _last_micros = micros_now;

    if(abs(micros_now - _last_fault_check_micros) >= _fault_check_interval) {
        _last_fault_check_micros = micros_now;       
        status_variables.fault = FaultStatus();
    }

    if(abs(micros_now - _last_status_micros) >= _status_interval) {
        _last_status_micros = micros_now;
        bitWrite(status_byte, DIRECTION_BIT, status_variables.direction); // Bit 0
        bitWrite(status_byte, FAULT_BIT, status_variables.fault); // Bit 1
        bitWrite(status_byte, PAUSED_BIT, status_variables.paused); // Bit 2
        bitWrite(status_byte, USE_RAMPING_BIT, status_variables.use_ramping); // Bit 3
        // bitWrite(status_byte, 4, ); // Bit 4
        bitWrite(status_byte, ENABLE_BIT, status_variables.enabled); // Bit 5
        bitWrite(status_byte, RUNNING_BIT, status_variables.running); // Bit 6
        bitWrite(status_byte, SLEEP_BIT, status_variables.sleep); // Bit 7
    }

    return job_done;
}
