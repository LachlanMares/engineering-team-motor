#ifndef QuadratureEncoder_h
#define QuadratureEncoder_h

#include "Arduino.h"

/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    ??    

Description:
    Interface for ABZ quadrature encoder  
*/

#define ENC_MAF_FILTER_LENGTH  10

struct encoder_struct {
  bool direction;
  float velocity_count;
  float velocity_radians;
  int angle_count;
  float angle_radians;
  long count;
  long delta;
  unsigned long errors;
};

class QuadratureEncoder{
        public:  
                QuadratureEncoder(unsigned long micros_period, int ppr, bool filter);            
                void interruptUpdateABExternal(bool a, bool b, bool z);
                void interruptUpdateZExternal(bool a, bool b);
                bool updateEncoderVelocity(unsigned long micros_now);
                bool getEncoderDirection();
                long getEncoderCount();
                long getEncoderDelta();
                int getEncoderAngleCount();
                float getEncoderVelocityRadians();
                float getEncoderAngleRadians();
                float getEncoderVelocity();
                long getEncoderErrorCount();
                encoder_struct encoder_status;
    
        private:
                bool previous_z;
                bool use_filter;
                uint8_t previous_state = 0;
                int pulses_per_revolution;
                int angle_count_int = 0;
                float update_dt = 0.0;
                float pulses_per_revolution_float;
                long previous_encoder_count = 0;
                unsigned long previous_micros_now = 0;
                unsigned long update_micros_period = 0;
                float filter_buffer[ENC_MAF_FILTER_LENGTH];
                
};

#endif