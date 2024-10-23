#include <stdio.h>
#include <unistd.h>
#include <syslog.h>

#include "fsm_middleware.h"


int main() {
  state_t cur_state = STATE_INIT;
  openlog("SM", LOG_PID | LOG_PERROR, LOG_USER);
  syslog(LOG_INFO, "Starting SM");
  do {
    cur_state = run_middleware(cur_state, NULL);
    usleep(500000);
  } while (cur_state != STATE_FATAL);
  run_middleware(cur_state, NULL);
  return 0;
}