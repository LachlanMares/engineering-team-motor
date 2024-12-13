#ifndef ScheduleMicro_h
#define ScheduleMicro_h

#include "Arduino.h"

class ScheduleMicro {

public:
        ScheduleMicro(unsigned long);
        ScheduleMicro(unsigned long, unsigned long);
        ScheduleMicro(unsigned long, unsigned long, unsigned long);
	ScheduleMicro(unsigned long, unsigned long, unsigned long, unsigned long);
        ScheduleMicro(unsigned long, unsigned long, unsigned long, unsigned long, unsigned long);
        void start();
        void stop();
        void update();
        void enableTask(int);
        void disableTask(int);
	void editTime(int, unsigned long);
        bool taskReady(int);
private:
        bool enable, enable_t0, enable_t1, enable_t2, enable_t3, enable_t4;
        bool task_0_ready, task_1_ready, task_2_ready, task_3_ready, task_4_ready;
        int num_of_tasks;
        unsigned long t0_period, t1_period, t2_period, t3_period, t4_period;
        unsigned long t0_micros, t1_micros, t2_micros, t3_micros, t4_micros;
};

#endif