// this file controls the motors via the orders of the navegation

// the order that the navegation can send
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>
//proportional controller variable
#define Kp (30/10000)

//global variable defining the task timer

extern volatile bool _is_update_control_ready ;
//getter for the flag of control
static inline bool  is_control_update_ready_n_clean(void){
  bool aux = _is_update_control_ready;
  _is_update_control_ready = false;
  return aux;  
}
// this function controls the robot receiving the desired position and the current
// state of the robot, the parameters are micro rad /s and micrometers/s
void update_control(int32_t linear_vel,int32_t ang_vel,int16_t left_enc, int16_t right_enc); 
#endif // CONTROL_H
