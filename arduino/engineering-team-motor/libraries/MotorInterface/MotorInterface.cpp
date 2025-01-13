#include "MotorInterface.h"

/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    ??    

Description:
    Interface for Texas Instruments DRV8825 Stepper motor driver and ABZ quadrature encoder  
*/


MotorInterface::MotorInterface(unsigned long update_period_us, int ppr, bool filter) : QuadratureEncoder(update_period_us, ppr, filter) {
    /*
        :param update_period_us: Period in microseconds used to update encoder velocity
        :param ppr: Number of encoder transitions per revolution for quadrature mode 4*number of pulses written on device
        :param filter: Enable/Disable moving average filter for encoder velocity
    */  
    
    _output_state = false;

    // Set all status variables to default
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

    // Set all command variables to default
    ClearCommandVariables();
}

void MotorInterface::Init(int direction_pin, int step_pin, int sleep_pin, int reset_pin, int fault_pin, int m0_pin, int m1_pin, int m2_pin, int enable_pin) { 
    /*
        :motor controller info -> https://www.pololu.com/product/2133

        :param direction_pin: Pin used to set motor diection CW/CCW
        :param step_pin: Pin used to rotate motor
        :param sleep_pin: Pin used to put mtor controller into low power mode
        :param reset_pin: Pin used to reset faults on motor controller
        :param fault_pin: Input used to determine if the motor controller is in fault
        :param m0_pin / m1_pin / m2_pin: The resolution (step size) selector inputs (MODE0, MODE1, and MODE2) enable selection from the six step resolutions
        :param enable_pin: Pin to Enable/Disable motor controller
    */  
    
    // Store hardware pin numbers
    _direction_pin = direction_pin;
    _step_pin = step_pin;
    _sleep_pin = sleep_pin;
    _reset_pin = reset_pin;
    _fault_pin = fault_pin;
    _m0_pin = m0_pin;
    _m1_pin = m1_pin; 
    _m2_pin = m2_pin;
    _enable_pin = enable_pin;

    // Assign hardware pins
    pinMode(_direction_pin, OUTPUT);
    pinMode(_step_pin, OUTPUT);
    pinMode(_sleep_pin, OUTPUT);
    pinMode(_reset_pin, OUTPUT);
    pinMode(_fault_pin, INPUT);
    pinMode(_m0_pin, OUTPUT);
    pinMode(_m1_pin, OUTPUT);
    pinMode(_m2_pin, OUTPUT);
    pinMode(_enable_pin, OUTPUT);

    // Set ouputs to default values
    digitalWrite(_direction_pin, DEFAULT_DIRECTION);
    digitalWrite(_step_pin, DEFAULT_STEP);
    digitalWrite(_sleep_pin, DEFAULT_SLEEP);
    digitalWrite(_reset_pin, DEFAULT_RESET);
    digitalWrite(_m0_pin, DEFAULT_M0);
    digitalWrite(_m1_pin, DEFAULT_M1);
    digitalWrite(_m2_pin, DEFAULT_M2);
    digitalWrite(_enable_pin, DEFAULT_ENABLE);
} 

void MotorInterface::ClearCommandVariables() {
    // Set all command variables to default
    command_variables.direction = false;
    command_variables.use_ramping = false;
    command_variables.microstep = 1;
    command_variables.job_id = 0;
    command_variables.ramp_scaler = DEFAULT_RAMP_SCALER;
    command_variables.ramping_steps = 0;
    command_variables.pulses = 0;
    command_variables.pulse_interval = DEFAULT_PULSE_INTERVAL;
    command_variables.pulse_on_period = DEFAULT_PULSE_ON_PERIOD;
}

void MotorInterface::Enable() {
    // Enable motor controller
    if (!status_variables.enabled) {
        status_variables.enabled = true;
        digitalWrite(_enable_pin, LOW);
    }
}

void MotorInterface::Disable() {
    // Disable motor controller
    if (status_variables.enabled) {
        status_variables.enabled = false;
        digitalWrite(_enable_pin, HIGH);
    }
}

void MotorInterface::Wake() {
    // Wake motor controller from sleep
    if (status_variables.sleep) {
        status_variables.sleep = false;
        digitalWrite(_sleep_pin, HIGH);
    }
}

void MotorInterface::Sleep() {
    // Put motor controller into low power mode
    if (!status_variables.sleep) {
        status_variables.sleep = true;
        digitalWrite(_sleep_pin, LOW);
    }
}

void MotorInterface::Reset() {
    // Reset motor controller
    digitalWrite(_reset_pin, LOW);
    delay(10);
    digitalWrite(_reset_pin, HIGH);
}

bool MotorInterface::FaultStatus() {
    status_variables.fault = false; // !digitalRead(_fault_pin);  # TODO sort this out
    return status_variables.fault;
}

void MotorInterface::DecodeMicroStep() {
    /* Set microstep pins 

    MODE0 	MODE1 	MODE2 	Microstep Resolution
    Low 	Low 	Low 	Full step
    High 	Low 	Low 	Half step
    Low 	High 	Low 	1/4 step
    High 	High 	Low 	1/8 step
    Low 	Low 	High 	1/16 step
    High 	Low 	High 	1/32 step
    Low 	High 	High 	1/32 step
    High 	High 	High 	1/32 step
    */

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
    // This is where jobs are requested
    if(FaultStatus()) {
        status_variables.running = false;
        Sleep();
        Disable();
        Reset();

    } else {
        // Copy command variables into status variables
        status_variables.running = true;
        status_variables.direction = command_variables.direction;
        status_variables.use_ramping = command_variables.use_ramping;
        status_variables.microstep = command_variables.microstep;
        status_variables.job_id = command_variables.job_id;
        status_variables.paused = false;
        // ramp scaler sets the ramp rate
        status_variables.ramp_scaler = (command_variables.ramp_scaler == 0) ? DEFAULT_RAMP_SCALER : command_variables.ramp_scaler;

        _output_state = false;

        // Reset some timer variables
        _last_pulse_on_micros = 0;
        _last_pulse_off_micros = 0;

        Enable();
        Wake();
        
        // Set direction and reset step pin
        digitalWrite(_direction_pin, status_variables.direction ? HIGH : LOW);
        digitalWrite(_step_pin, LOW);
        
        // Set microstep
        DecodeMicroStep();

        // Sanity check requested variables
        status_variables.pulse_interval = (command_variables.pulse_interval > MINIMUM_PULSE_INTERVAL && command_variables.pulse_interval < MAXIMUM_PULSE_INTERVAL) ? command_variables.pulse_interval : DEFAULT_PULSE_INTERVAL;
        status_variables.pulse_on_period = (command_variables.pulse_on_period < command_variables.pulse_interval && command_variables.pulse_on_period != 0) ? command_variables.pulse_on_period : (long)(status_variables.pulse_interval/2);
        status_variables.pulses_remaining = command_variables.pulses;

        // If ramping has been selected calculate rates and start/end points
        if(status_variables.use_ramping) {
            if (command_variables.ramping_steps == 0) {
                command_variables.ramping_steps = DEFAULT_RAMP_STEPS;
            }

            if(2 * command_variables.ramping_steps < status_variables.pulses_remaining) {
                // There are enough steps remaining to use specified ramp 
                status_variables.ramp_up_stop = status_variables.pulses_remaining - command_variables.ramping_steps;
                status_variables.ramp_down_start = command_variables.ramping_steps;

            } else {
                // Set ramp up/down to half the number of total steps
                status_variables.ramp_up_stop = (long)(status_variables.pulses_remaining / 2);
                status_variables.ramp_down_start = status_variables.ramp_up_stop - 1;
            }

            status_variables.ramp_up_interval = status_variables.pulse_interval * command_variables.ramp_scaler;
            status_variables.ramp_pulse_interval = status_variables.ramp_up_interval;
            status_variables.ramp_down_interval = status_variables.pulse_interval;
            status_variables.ramp_interval_step = (long)((status_variables.ramp_up_interval - status_variables.ramp_down_interval) / command_variables.ramping_steps);

        } else {
            // Ramping not selected reset these variables
            status_variables.ramp_up_stop = 0;
            status_variables.ramp_down_start = 0;
            status_variables.ramp_up_interval = 0;
            status_variables.ramp_down_interval = 0;
            status_variables.ramp_interval_step = 0;
            status_variables.ramp_pulse_interval = 0;
        }
    }

    ClearCommandVariables();
}

void MotorInterface::PauseJob() {
    // Pause current job
    status_variables.paused = true;
}

void MotorInterface::ResumeJob() {
    // Resume current job
    status_variables.paused = false;
    _last_pulse_on_micros = 0;
    _last_pulse_off_micros = 0;
}

void MotorInterface::CancelJob() {
    // Cancel current job
    status_variables.running = false;
    _output_state = false;
    status_variables.pulses_remaining = 0;
    status_variables.enabled = false;

    ResetJobId();
    digitalWrite(_step_pin, LOW);
}

void MotorInterface::ResetJobId() {
    status_variables.job_id = 0;
}

bool MotorInterface::Update(unsigned long micros_now) {
    /*
    This is the main function used to pule the motor at the correct intervals and update status variables
    
    param: micros_now: Current reading of the Micros() function
    */

    updateEncoderVelocity(micros_now);

    bool job_done = false;

    // Check that motor controller is enabled
    if(status_variables.enabled) {
        
        // Check for active job, active job is not paused and controller is not in fault
        if(status_variables.running && !status_variables.paused && !status_variables.fault) {
            
            if(status_variables.pulses_remaining > 0) {

                if(status_variables.use_ramping) {
                    
                    if(!_output_state) {
                        // Output pin is LOW, waiting for the timer to expire before going HIGH
                        if(abs(micros_now - _last_pulse_on_micros) >= status_variables.ramp_pulse_interval) {

                            _last_pulse_on_micros = micros_now;
                            _last_pulse_off_micros = micros_now;
                            _output_state = true;
                            digitalWrite(_step_pin, HIGH);

                            // Ramping up 
                            if(status_variables.pulses_remaining > status_variables.ramp_up_stop) {
                                status_variables.ramp_up_interval -= status_variables.ramp_interval_step;
                                status_variables.ramp_pulse_interval = status_variables.ramp_up_interval;
                            
                            // Ramping down
                            } else if(status_variables.pulses_remaining < status_variables.ramp_down_start) {
                                status_variables.ramp_down_interval += status_variables.ramp_interval_step;
                                status_variables.ramp_pulse_interval = status_variables.ramp_down_interval;

                            // Full speed
                            } else {
                                status_variables.ramp_pulse_interval = status_variables.pulse_interval;
                            }
                        }
                    
                    
                    } else {
                        // Output pin is HIGH, waiting to go LOW after timer expires
                        if(abs(micros_now - _last_pulse_off_micros) >= status_variables.pulse_on_period) {
                            digitalWrite(_step_pin, LOW);
                            _output_state = false;
                            status_variables.pulses_remaining--;
                        }
                    }

                } else {
                    if(!_output_state) {
                        // Output pin is LOW, waiting for the timer to expire before going HIGH
                        if(abs(micros_now - _last_pulse_on_micros) >= status_variables.pulse_interval) {
                            digitalWrite(_step_pin, HIGH);
                            _last_pulse_on_micros = micros_now;
                            _last_pulse_off_micros = micros_now;
                            _output_state = true;
                        }
                        
                    // Output pin is HIGH, waiting to go LOW after timer expires
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

void MotorInterface::UpdateStatus() {
    // Pack status bits into a status byte 
    bitWrite(status_byte, DIRECTION_BIT, status_variables.direction); // Bit 0
    bitWrite(status_byte, FAULT_BIT, status_variables.fault); // Bit 1
    bitWrite(status_byte, PAUSED_BIT, status_variables.paused); // Bit 2
    bitWrite(status_byte, USE_RAMPING_BIT, status_variables.use_ramping); // Bit 3
    // bitWrite(status_byte, 4, ); // Bit 4
    bitWrite(status_byte, ENABLE_BIT, status_variables.enabled); // Bit 5
    bitWrite(status_byte, RUNNING_BIT, status_variables.running); // Bit 6
    bitWrite(status_byte, SLEEP_BIT, status_variables.sleep); // Bit 7
}


