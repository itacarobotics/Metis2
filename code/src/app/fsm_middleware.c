/******************************************************************************
Finite State Machine
Project: ../../assets/gv_fsm/_fsm_middleware.dot
Description: fsm_middleware

Generated by gv_fsm ruby gem, see https://rubygems.org/gems/gv_fsm
gv_fsm version 0.3.7
Generation date: 2024-09-22 19:51:29 +0200
Generated from: ../../assets/gv_fsm/_fsm_middleware.dot
The finite state machine has:
  5 states
  3 transition functions
******************************************************************************/

#include <syslog.h>
#include "fsm_middleware.h"

// SEARCH FOR Your Code Here FOR CODE INSERTION POINTS!

// GLOBALS
// State human-readable names
const char *state_names[] = {"init", "consume", "compute", "produce", "fatal"};


/*** USER CODE BEGIN GLOBAL ***/
gcode_t gcode_line;
bfr_gcode_t bfr_gcode_cmds;
bfr_gcode_t bfr_robot_cmds;
/*** USER CODE END GLOBAL ***/


// List of state functions
state_func_t *const state_table[NUM_STATES] = {
  do_init,    // in state init
  do_consume, // in state consume
  do_compute, // in state compute
  do_produce, // in state produce
  do_fatal,   // in state fatal
};

// Table of transition functions
transition_func_t *const transition_table[NUM_STATES][NUM_STATES] = {
  /* states:     init               , consume            , compute            , produce            , fatal               */
  /* init    */ {NULL               , NULL               , NULL               , NULL               , NULL               }, 
  /* consume */ {NULL               , NULL               , NULL               , NULL               , NULL               }, 
  /* compute */ {NULL               , NULL               , NULL               , NULL               , handle_fatal_error }, 
  /* produce */ {NULL               , NULL               , NULL               , NULL               , NULL               }, 
  /* fatal   */ {NULL               , NULL               , NULL               , NULL               , NULL               }, 
};

/*  ____  _        _       
 * / ___|| |_ __ _| |_ ___ 
 * \___ \| __/ _` | __/ _ \
 *  ___) | || (_| | ||  __/
 * |____/ \__\__,_|\__\___|
 *                         
 *   __                  _   _                 
 *  / _|_   _ _ __   ___| |_(_) ___  _ __  ___ 
 * | |_| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 * |  _| |_| | | | | (__| |_| | (_) | | | \__ \
 * |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
 */                                             

// Function to be executed in state init
// valid return states: STATE_CONSUME
state_t do_init(state_data_t *data) {
  state_t next_state = STATE_CONSUME;

  syslog(LOG_INFO, "[FSM] In state init");
  
  /*** USER CODE BEGIN INIT ***/
  bfr_init(&bfr_gcode_cmds, 8);
  bfr_init(&bfr_robot_cmds, 8);
  /*** USER CODE END INIT ***/

  switch (next_state) {
    case STATE_CONSUME:
      break;
    default:
      syslog(LOG_WARNING, "[FSM] Cannot pass from init to %s, remaining in this state", state_names[next_state]);
      next_state = NO_CHANGE;
  }
  
  return next_state;
}


// Function to be executed in state consume
// valid return states: NO_CHANGE, STATE_CONSUME, STATE_COMPUTE, STATE_PRODUCE
state_t do_consume(state_data_t *data) {
  state_t next_state = NO_CHANGE;

  syslog(LOG_INFO, "[FSM] In state consume");
  
  /*** USER CODE BEGIN CONSUME ***/
  int32_t rc;
  
  // get item from buffer
  rc = bfr_consume(&bfr_gcode_cmds, &gcode_line);
  if (rc != MOD_RET_OK) {   // buffer empty or busy
    return next_state;
  }

  // handle new item
  switch (gcode_line.cmd)
  {
  case G01:
  case G02:
  case G03:
    RETCHECK(tg_set_next_trajectory(gcode_line), STATE_COMPUTE);
    break;

  case G90:
  case G91:
    next_state = NO_CHANGE;
    tg_set_positioning(gcode_line.cmd);
    break;

  case G04:
  case G28:
  case M04:
  case M05:
  case M17:
  case M18:
    next_state = STATE_PRODUCE;
    break;

  default:
    break;
  }
  /*** USER CODE END CONSUME ***/  

  switch (next_state) {
    case NO_CHANGE:
    case STATE_CONSUME:
    case STATE_COMPUTE:
    case STATE_PRODUCE:
      break;
    default:
      syslog(LOG_WARNING, "[FSM] Cannot pass from consume to %s, remaining in this state", state_names[next_state]);
      next_state = NO_CHANGE;
  }

  return next_state;
}


// Function to be executed in state compute
// valid return states: STATE_PRODUCE, STATE_FATAL
state_t do_compute(state_data_t *data) {
  state_t next_state = STATE_PRODUCE;
  
  syslog(LOG_INFO, "[FSM] In state compute");
  
  /*** USER CODE BEGIN COMPUTE ***/
  int32_t rc;
  
  rc = tg_get_via_point(&gcode_line);
  switch (rc)
  {
  case MP_ERR_END_OF_TRAJECTORY:
    next_state = STATE_CONSUME;
    break;
  case MOD_RET_OK:
    next_state = STATE_PRODUCE;
  default:
    next_state = STATE_FATAL;
    break;
  }
  /*** USER CODE END COMPUTE ***/

  switch (next_state) {
    case STATE_PRODUCE:
    case STATE_FATAL:
      break;
    default:
      syslog(LOG_WARNING, "[FSM] Cannot pass from compute to %s, remaining in this state", state_names[next_state]);
      next_state = NO_CHANGE;
  }
  
  return next_state;
}


// Function to be executed in state produce
// valid return states: NO_CHANGE, STATE_CONSUME, STATE_COMPUTE, STATE_PRODUCE
state_t do_produce(state_data_t *data) {
  state_t next_state = NO_CHANGE;
  
  syslog(LOG_INFO, "[FSM] In state produce");
  
  /*** USER CODE BEGIN PRODUCE ***/
  int32_t rc;
  rc = bfr_produce(&bfr_robot_cmds, gcode_line);
  if (rc != MOD_RET_OK) {         // buffer is full
    return next_state;
  }

  // handle item
  switch (gcode_line.cmd)
  {
  case G01:
  case G02:
  case G03:
    next_state = STATE_COMPUTE;
    break;

  case G04:
  case G28:
  case M04:
  case M05:
  case M17:
  case M18:
    next_state = STATE_CONSUME;
    break;
  
  default:
    break;
  }

  /*** USER CODE END PRODUCE ***/

  switch (next_state) {
    case NO_CHANGE:
    case STATE_CONSUME:
    case STATE_COMPUTE:
    case STATE_PRODUCE:
      break;
    default:
      syslog(LOG_WARNING, "[FSM] Cannot pass from produce to %s, remaining in this state", state_names[next_state]);
      next_state = NO_CHANGE;
  }
  
  return next_state;
}


// Function to be executed in state fatal
// valid return states: NO_CHANGE
state_t do_fatal(state_data_t *data) {
  state_t next_state = NO_CHANGE;
  
  syslog(LOG_INFO, "[FSM] In state fatal");

  /*** USER CODE BEGIN FATAL ***/


  /*** USER CODE END FATAL ***/
  
  switch (next_state) {
    case NO_CHANGE:
      break;
    default:
      syslog(LOG_WARNING, "[FSM] Cannot pass from fatal to %s, remaining in this state", state_names[next_state]);
      next_state = NO_CHANGE;
  }
  
  return next_state;
}


/*  _____                    _ _   _              
 * |_   _| __ __ _ _ __  ___(_) |_(_) ___  _ __   
 *   | || '__/ _` | '_ \/ __| | __| |/ _ \| '_ \
 *   | || | | (_| | | | \__ \ | |_| | (_) | | | | 
 *   |_||_|  \__,_|_| |_|___/_|\__|_|\___/|_| |_| 
 *                                                
 *   __                  _   _                 
 *  / _|_   _ _ __   ___| |_(_) ___  _ __  ___ 
 * | |_| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 * |  _| |_| | | | | (__| |_| | (_) | | | \__ \
 * |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
 */

// This function is called in 1 transition:
// 1. from compute to fatal
void handle_fatal_error(state_data_t *data) {
  syslog(LOG_INFO, "[FSM] State transition handle_fatal_error");
  
  /*** USER CODE BEGIN HANDLE_FATAL_ERROR ***/


  /*** USER CODE END HANDLE_FATAL_ERROR ***/
  
  return;
}


/*  ____  _        _        
 * / ___|| |_ __ _| |_ ___  
 * \___ \| __/ _` | __/ _ \
 *  ___) | || (_| | ||  __/ 
 * |____/ \__\__,_|\__\___| 
 *                          
 *                                              
 *  _ __ ___   __ _ _ __   __ _  __ _  ___ _ __ 
 * | '_ ` _ \ / _` | '_ \ / _` |/ _` |/ _ \ '__|
 * | | | | | | (_| | | | | (_| | (_| |  __/ |   
 * |_| |_| |_|\__,_|_| |_|\__,_|\__, |\___|_|   
 *                              |___/           
 */

state_t run_middleware(state_t cur_state, state_data_t *data) {
  state_t new_state = state_table[cur_state](data);
  if (new_state == NO_CHANGE)
    new_state = cur_state;

  transition_func_t *transition = transition_table[cur_state][new_state];
  if (transition)
    transition(data);

  return new_state;
};

#ifdef TEST_MAIN
#include <unistd.h>
int main() {
  state_t cur_state = STATE_INIT;
  openlog("SM", LOG_PID | LOG_PERROR, LOG_USER);
  syslog(LOG_INFO, "Starting SM");
  do {
    cur_state = run_state(cur_state, NULL);
    sleep(1);
  } while (cur_state != STATE_FATAL);
  run_state(cur_state, NULL);
  return 0;
}
#endif
