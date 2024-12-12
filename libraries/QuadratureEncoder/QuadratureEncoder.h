#ifndef QuadratureEncoder_h
#define QuadratureEncoder_h

#include "Arduino.h"

struct encoder_struct {
  bool direction;
  float velocity;
  long count;
  long delta;
  unsigned long errors;
};

class QuadratureEncoder{
        public:  
                QuadratureEncoder(unsigned long micros_period);            
               
                encoder_struct encoder_status;

                void interruptUpdateExternal(bool a, bool b);
                bool updateEncoder(unsigned long micros_now);
                bool getEncoderDirection();
                long getEncoderCount();
                long getEncoderDelta();
                float getEncoderVelocity();
                void setEncoderCount(long);
                long getEncoderErrorCount();

                encoder_struct getEncoderStatus();
    
        private:
                uint8_t encoder_a_pin;
                uint8_t encoder_b_pin;
                uint8_t previous_state = 0;
                float update_dt = 0.0;
                long previous_encoder_count = 0;
                unsigned long previous_micros_now = 0;
                unsigned long update_micros_period = 0;
};

#endif