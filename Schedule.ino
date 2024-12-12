void initialiseScheduler() {
  scheduler.start();  
  scheduler.enableTask(PRINT_TASK_ID);
  scheduler.enableTask(JOB_TASK_ID);
}

void updateScheduler() {
  scheduler.update();
}