#include "QuadratureEncoder.h"


QuadratureEncoder::QuadratureEncoder(unsigned long encoder_update_period_us, int ppr, bool filter) {
  update_micros_period = encoder_update_period_us;
  update_dt = (float)(update_micros_period) * 1e-6;
  previous_micros_now = micros();
  previous_encoder_count = 0;
  pulses_per_revolution = ppr;
  pulses_per_revolution_float = (float)(ppr);
  use_filter = filter;
  previous_z = false;
  
  encoder_status.direction = true;
  encoder_status.velocity_count = 0.0;
  encoder_status.angle_count = 0;
  encoder_status.velocity_radians = 0.0;
  encoder_status.angle_radians = 0.0;
  encoder_status.count = 0;
  encoder_status.delta = 0;
  encoder_status.errors = 0;
}


void QuadratureEncoder::interruptUpdateZExternal(bool a, bool b) {
  //                           _______         _______       
  //               PinA ______|       |_______|       |______ PinA
  // negative <---         _______         _______         __      --> positive
  //               PinB __|       |_______|       |_______|   PinB
  //                       ___________        
  //               PinZ __|           |____________________   PinZ 
  //
}

void QuadratureEncoder::interruptUpdateABExternal(bool a, bool b, bool z) {
  //                           _______         _______       
  //               PinA ______|       |_______|       |______ PinA
  // negative <---         _______         _______         __      --> positive
  //               PinB __|       |_______|       |_______|   PinB
  //                           ___________        
  //               PinZ ______|           |__________________   PinZ 
  //
  //	new	  new	  old	  old
  //	pinB	pinA	pinB	pinA	Result
  //	----	----	----	----	------
  //	0	0	0	0	  no movement / error
  //	0	0	0	1	  +1
  //	0	0	1	0	  -1
  //  0	0	1	1   +2

  //	0	1	0	0	  -1
  //	0	1	0	1	  no movement / error
  //	0	1	1	1	  +1
  //  0	1	1	0   -2

  //	1	0	0	0	  +1
  //	1	0	1	0	  no movement / error
  //	1	0	1	1	  -1
  //  1	0	0	1	  -2

  //	1	1	0	1	  -1
  //	1	1	1	0	  +1
  //	1	1	1	1	  no movement / error
  //  1	1	0	0	  +2

  uint8_t state = (previous_state >> 2) & 0x03; 

  bitWrite(state, 3, a);
  bitWrite(state, 2, b);

  switch(state) {
    case 0b0001: case 0b0111: case 0b1000: case 0b1110:
      encoder_status.count++;
      encoder_status.angle_count++;
      encoder_status.direction = true;
      previous_state = state;

      if (z && !previous_z) {
        encoder_status.angle_count = 0;
      }
      break;
    
    case 0b0010: case 0b0100: case 0b1011: case 0b1101: 
      encoder_status.count--;
      encoder_status.angle_count--;
      encoder_status.direction = false;
      previous_state = state;

      if (z && !previous_z) {
        encoder_status.angle_count = pulses_per_revolution - 4;
      }
      break;

    default:
      encoder_status.errors++;
      break;
  }

  previous_z = z;
  encoder_status.angle_count = encoder_status.angle_count % pulses_per_revolution;
  
  if (encoder_status.angle_count < 0) {
    encoder_status.angle_count = pulses_per_revolution + encoder_status.angle_count;
  }
}

bool QuadratureEncoder::updateEncoderVelocity(unsigned long micros_now) {
  if (abs(micros_now - previous_micros_now) >= update_micros_period) {
    encoder_status.delta = -(previous_encoder_count - encoder_status.count);  // Get number of pulses since last update
    encoder_status.velocity_count = (float)(encoder_status.delta) / update_dt;  // Pulses per second
    
    if (use_filter) {
      float velocity_sum = 0.0;

      // Basic moving average filter, move previous readings
      for (uint8_t i=1; i<ENC_MAF_FILTER_LENGTH; i++) {
          filter_buffer[i-1] = filter_buffer[i]; 
          velocity_sum += filter_buffer[i-1];
      }

      filter_buffer[ENC_MAF_FILTER_LENGTH-1] = ((float)(encoder_status.delta) / pulses_per_revolution_float) * 6.28318531;
      encoder_status.velocity_radians = (velocity_sum + filter_buffer[ENC_MAF_FILTER_LENGTH-1]) / ENC_MAF_FILTER_LENGTH;

    } else {
      encoder_status.velocity_radians = ((float)(encoder_status.delta) / pulses_per_revolution_float) * 6.28318531;
    }
    
    previous_encoder_count = encoder_status.count;  // Store variables for next time
    previous_micros_now = micros_now;

    return true;
    
  } else {

    return false;
  }
}

long QuadratureEncoder::getEncoderCount() {
  return encoder_status.count;
}

bool QuadratureEncoder::getEncoderDirection() {
  return encoder_status.direction;
}

int QuadratureEncoder::getEncoderAngleCount() { 
  return encoder_status.angle_count;
}

float QuadratureEncoder::getEncoderVelocityRadians() {
  return encoder_status.velocity_radians;
}

float QuadratureEncoder::getEncoderAngleRadians() {
  return ((float)(encoder_status.angle_count) / pulses_per_revolution_float) * 6.28318531;
}

long QuadratureEncoder::getEncoderDelta() {
  return encoder_status.delta;
}

float QuadratureEncoder::getEncoderVelocity() {
  return encoder_status.velocity_count;
}

long QuadratureEncoder::getEncoderErrorCount() {
  return encoder_status.errors;
}