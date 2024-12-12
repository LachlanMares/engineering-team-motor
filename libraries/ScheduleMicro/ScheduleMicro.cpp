#include "ScheduleMicro.h"	

ScheduleMicro::ScheduleMicro(unsigned long period0) {
  enable = false;
  enable_t0 = false;
  enable_t1 = false;
  enable_t2 = false;
  enable_t3 = false;
  enable_t4 = false;
  task_0_ready = false;
  t0_period = period0;
  num_of_tasks = 1;
}

ScheduleMicro::ScheduleMicro(unsigned long period0, unsigned long period1) {
  enable = false;
  enable_t0 = false;
  enable_t1 = false;
  enable_t2 = false;
  enable_t3 = false;
  enable_t4 = false;
  task_0_ready = false;
  t0_period = period0;
  task_1_ready = false;
  t1_period = period1;
  num_of_tasks = 2;
}

ScheduleMicro::ScheduleMicro(unsigned long period0, unsigned long period1, unsigned long period2) {
  enable = false;
  enable_t0 = false;
  enable_t1 = false;
  enable_t2 = false;
  enable_t3 = false;
  enable_t4 = false;
  task_0_ready = false;
  t0_period = period0;
  task_1_ready = false;
  t1_period = period1;
  task_2_ready = false;
  t2_period = period2;
  num_of_tasks = 3;
}

ScheduleMicro::ScheduleMicro(unsigned long period0, unsigned long period1, unsigned long period2, unsigned long period3) {
  enable = false;
  enable_t0 = false;
  enable_t1 = false;
  enable_t2 = false;
  enable_t3 = false;
  enable_t4 = false;
  task_0_ready = false;
  t0_period = period0;
  task_1_ready = false;
  t1_period = period1;
  task_2_ready = false;
  t2_period = period2;
  task_3_ready = false;
  t3_period = period3;  
  num_of_tasks = 4;
}

ScheduleMicro::ScheduleMicro(unsigned long period0, unsigned long period1, unsigned long period2, unsigned long period3, unsigned long period4) {
  enable = false;
  enable_t0 = false;
  enable_t1 = false;
  enable_t2 = false;
  enable_t3 = false;
  enable_t4 = false;
  task_0_ready = false;
  t0_period = period0;
  task_1_ready = false;
  t1_period = period1;
  task_2_ready = false;
  t2_period = period2;
  task_3_ready = false;
  t3_period = period3;
  task_4_ready = false;
  t4_period = period4;    
  num_of_tasks = 5;
}

void ScheduleMicro::start() {
  unsigned long micros_now = micros();
  t0_micros = micros_now;
  t1_micros = micros_now;
  t2_micros = micros_now;
  t3_micros = micros_now;
  t4_micros = micros_now;
  enable = true;  
}

void ScheduleMicro::stop() {
  enable = false;
}

void ScheduleMicro::update() {
  if(enable){
    unsigned long micros_now = micros();

    if(enable_t0){
      if((micros_now - t0_micros) >= t0_period) {                                    
        t0_micros = micros_now;
        task_0_ready = true;
      }
    }

    if(enable_t1){
      if((micros_now - t1_micros) >= t1_period){                                    
        t1_micros = micros_now;
        task_1_ready = true;
      }
    }
    
    if(enable_t2){
      if((micros_now - t2_micros) >= t2_period){                                    
        t2_micros = micros_now;
        task_2_ready = true;
      }
    }
    
    if(enable_t3){
      if((micros_now - t3_micros) >= t3_period){                                  
        t3_micros = micros_now;
        task_3_ready = true;
      }
    }
      
    if(enable_t4){
      if((micros_now - t4_micros) >= t4_period){                                  
        t4_micros = micros_now;
        task_4_ready = true;
      }
    }   
  }
} 

void ScheduleMicro::enableTask(int TaskNum) {
  if (TaskNum < num_of_tasks) {
    switch(TaskNum) {
      case 0:
        enable_t0 = true;
        t0_micros = micros();
        break;
      case 1:
        enable_t1 = true;
        t1_micros = micros();
        break;    
      case 2:
        enable_t2 = true;
        t2_micros = micros();
        break;    
      case 3:
        enable_t3 = true;
        t3_micros = micros();
        break;   
      case 4:
        enable_t4 = true;
        t4_micros = micros();
        break;
    }      
  }
}

void ScheduleMicro::disableTask(int TaskNum) {
  if (TaskNum < num_of_tasks) {
    switch(TaskNum){
      case 0:
        enable_t0 = false;
        break;
      case 1:
        enable_t1 = false;
        break;    
      case 2:
        enable_t2 = false;
        break;    
      case 3:
        enable_t3 = false;
        break;  
      case 4:
        enable_t4 = false;
        break;  
    }    
  }
}

bool ScheduleMicro::taskReady(int TaskNum) {
  bool RetVal = false;
  if (TaskNum < num_of_tasks) {
    switch(TaskNum){
      case 0:
        if(task_0_ready) {
          task_0_ready = false;
          RetVal = true;
        }
        break;
      case 1:
        if(task_1_ready) {
          task_1_ready = false;
          RetVal = true;
        }
        break;    
      case 2:
        if(task_2_ready) {
          task_2_ready = false;
          RetVal = true;
        }
        break;    
      case 3:
        if(task_3_ready) {
          task_3_ready = false;
          RetVal = true;
        }
        break;  
      case 4:
        if(task_4_ready) {
          task_4_ready = false;
          RetVal = true;
        }
        break;
    }    
  }
  return RetVal;
}  

void ScheduleMicro::editTime(int TaskNum, unsigned long Period) {
  if (TaskNum < num_of_tasks && Period > 0) {
    switch(TaskNum) {
      case 0:
        t0_period = Period;
        break;
      case 1:
        t1_period = Period;
        break;    
      case 2:
        t2_period = Period;
        break;    
      case 3:
        t3_period = Period;
        break;  
      case 4:
        t4_period = Period;
        break;
    }
  }
}  




