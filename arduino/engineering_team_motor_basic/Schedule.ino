void initialiseScheduler() {
  scheduler.start();  
  scheduler.enableTask(PRINT_TASK_ID);
  scheduler.enableTask(JOB_INTERVAL_TASK_ID);
  scheduler.enableTask(FAULT_CHECK_TASK_ID);
}

void updateScheduler() {
  scheduler.update();
}