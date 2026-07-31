// this timer will be the responsible of determine the frequency of each of the
// task in the
//  robot, we will be using TIMER_B since it has 7 Compare registers and
//  therefore can make use of the fact we dont really need a counter for each
//  taask and instead the hardware takes care of everything. ( if we had more
//  than 7 tasks we would need to make a software counter instead)
//
//
//
#ifndef TASK_TIMER_H
#define TASK_TIMER_H
#include "config.h"
#include <msp430.h>
#define ACLK_HZ 32768UL
// Task frequency (using the 32 768 Hz clock)
//
// front distance sensor frecuency
#define FRONT_DS_CYCLES (uint16_t)((FRONT_DS_MS * ACLK_HZ) / 1000UL)
// left and right sensor frequency
#define LEFT_RIGHT_DS_CYCLES (uint16_t)((LEFT_RIGHT_DS_MS * ACLK_HZ) / 1000UL)
#define POSITION_UPDATE_CYCLES                                                 \
  (uint16_t)((POSITION_UPDATE_MS * ACLK_HZ) / 1000UL)

// configuration of the Master clock that determines the frequency of the tasks
void config_task_timer();

#endif
