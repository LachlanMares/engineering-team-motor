#include "QuadratureEncoder.h"


QuadratureEncoder::QuadratureEncoder(unsigned long encoder_update_period_us) {
  update_micros_period = encoder_update_period_us;
  update_dt = (float)(update_micros_period) * 1e-6;
  previous_micros_now = micros();

  encoder_status.direction = true;
  encoder_status.velocity = 0.0;
  encoder_status.count = 0;
  encoder_status.delta = 0;
  encoder_status.errors = 0;
}

void QuadratureEncoder::interruptUpdateExternal(bool a, bool b) {
  //                           _______         _______       
  //               PinA ______|       |_______|       |______ PinA
  // negative <---         _______         _______         __      --> positive
  //               PinB __|       |_______|       |_______|   PinB
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
      encoder_status.direction = true;
      previous_state = state;
      break;
    
    case 0b0010: case 0b0100: case 0b1011: case 0b1101: 
      encoder_status.count--;
      encoder_status.direction = false;
      previous_state = state;
      break;

    default:
      encoder_status.errors++;
      break;
  }
}

bool QuadratureEncoder::updateEncoder(unsigned long micros_now) {
  if (abs(micros_now - previous_micros_now) >= update_micros_period) {
    encoder_status.delta = -(previous_encoder_count - encoder_status.count);  // Get number of pulses since last update
    encoder_status.velocity = (float)(encoder_status.delta) / update_dt;  // Pulses per second
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

long QuadratureEncoder::getEncoderDelta() {
  return encoder_status.delta;
}

float QuadratureEncoder::getEncoderVelocity() {
  return encoder_status.velocity;
}

void QuadratureEncoder::setEncoderCount(long new_encoder_count) {
  encoder_status.count = new_encoder_count;
  previous_encoder_count = new_encoder_count;
}

long QuadratureEncoder::getEncoderErrorCount() {
  return encoder_status.errors;
}

encoder_struct QuadratureEncoder::getEncoderStatus() {
  return encoder_status;
}