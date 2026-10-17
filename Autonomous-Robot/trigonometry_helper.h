// sin and cos
// we are changing to BAM representation , an angle is a uint16_t that represents the circle,
// being 0 = 0 degrees , 32768 = 180 degrees, 
// this is super useful so the number naturally wraps around and we can eliminate the
// comprobations if the angles is out of the 0 - 360 circle since the numbers does it automatically
#ifndef TRIGONOMETRY_HELPER_H
#define TRIGONOMETRY_HELPER_H
#include <stdint.h>

 

typedef uint16_t ang_unit;

//angle look up table , it holds the values of the angles in the Q16.16 format
extern const int32_t look_up_table[4096];

//this function receives the angle in the angular unit and returs the value between -65536 ( -1 ) and 65536(1)
static inline int32_t Q16_sin(ang_unit ang){
  //dividing the angle by 16 (4 bitshift) since the angle is a 2^16 and the lut has 2^12 positions
    return look_up_table[ang >> 4];
}
#define _90_DEG 16384
//the same as the sin function but adding 90 degrees to the angle
static inline int32_t Q16_cos(ang_unit ang){
  return look_up_table[(ang + _90_DEG) >> 4];
}

#endif
