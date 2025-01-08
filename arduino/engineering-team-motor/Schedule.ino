void initialiseScheduler() {
  scheduler.start();  
  scheduler.enableTask(FAULT_CHECK_TASK_ID);
  scheduler.enableTask(STATUS_MESSAGE_TASK_ID);
  scheduler.enableTask(MOTOR_FEEDBACK_TASK_ID);
}

void updateScheduler() {
  scheduler.update();
}