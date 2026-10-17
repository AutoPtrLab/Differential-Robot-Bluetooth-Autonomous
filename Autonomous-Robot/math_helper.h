// math helper for all the arithemtical operations scaled to 2^16
//  this includes scalars and vectors
#ifndef MATH_HELPER_H
#define MATH_HELPER_H

#include <stdint.h>
//====================
// -------CONSTANTS---
// =====================

// PI in Q16 scale
#define Q16_PI 205887

//conversion from int, the number has to be maximum a 16 bits number
#define INT_TO_Q16(x)  (((q16_t)(x)) << 16)
//conversion from float
#define FLOAT_TO_Q16(x) ((q16_t)((x) * 65536.0f))

typedef int32_t q16_t;


//==========================
// -----SCALARS------------
// ========================g

// this sections does all the operatoins between scalars that have been multiyplied by 2^16

// even if the sum and substraction is the same , implement the funcitons to make explicit that you
// are working with scaled numbers we have to be careful since the sum and substraction can overflow

static inline q16_t Q16_add(q16_t a, q16_t b) { return a + b; }
// substrats b from a
static inline q16_t Q16_sub(q16_t a, q16_t b) { return a - b; }
// multipliaction between two Q16
q16_t Q16_mult(q16_t a, q16_t b);

//division between two q16_t
q16_t Q16_div(q16_t dividen , q16_t divisor);
//division between a q16_t and a regular decimal number (this serves to divide a q16 not needing to scale the divisor)
// and returns the q16_t
static inline q16_t Q16_div_nat(q16_t q,int32_t divisor){
    return (q16_t)(((int64_t)q << 16) / divisor);
}






// =========================
// ---------VECTORS---------
//=========================
// Vector with two components that have been scaled with 2^16

typedef struct {
    q16_t x;
    q16_t y;
} Vec2;

// sum between two vectors
static inline Vec2 VEC_sum(const Vec2*  a, const Vec2* b){
    return (Vec2){a->x + b->x,a->y+b->y};
}

// substraction between Vec
static inline Vec2 VEC_sub(const Vec2 *  a,const Vec2* b){
    return (Vec2){a->x - b->x,a->y - b->y};
}

// multiplication by an scalar
Vec2 VEC_scalar_mut(const Vec2* vec, q16_t scalar);
// lenght of the vector
q16_t VEC_length(const Vec2* vec);
// lenght of the vector Sq ( No square root)
// WARNING : this function returns a Q32.32 since is squared, is only valid to
// compare it with other squared numbers
int64_t VEC_lenght_sq(const Vec2* vec);
// scalar product 
q16_t VEC_dot(const Vec2* a,const Vec2* b );
#endif
