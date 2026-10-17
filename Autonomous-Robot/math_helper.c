#include "math_helper.h"
#include <stdint.h>

q16_t Q16_mult(q16_t a, q16_t b) {

    return (((int64_t)a * (int64_t)b) >> 16);
}

q16_t Q16_div(q16_t dividen, q16_t divisor){

    return (q16_t)(((int64_t)dividen << 16) / divisor);    
}

//helper that receives a number Q32.32 (NOT AN Q16.16) and
// finds the sqrt in the Q16 format. this function is a helper
// for the lenghts functions
static q16_t Q16_sqrt(int64_t num){

    if (num <= 0) {
        return 0; 
    }

    uint64_t n = (uint64_t)num;
    uint64_t res = 0;
    uint64_t bit = 1ULL << 62; 
    //adjustment for the first guess    
    while (bit > n) {
        bit >>= 2;
    }
    
    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (q16_t)res;
}

Vec2 VEC_scalar_mut(const Vec2* vec, int32_t scalar){
    
    return (Vec2){Q16_mult(vec->x, scalar),Q16_mult(vec->y, scalar)};
}

int64_t VEC_length_sq(const Vec2* vec){

   return  (int64_t)(((int64_t)vec->x * (int64_t)vec->x) + ((int64_t)vec->y * (int64_t)vec->y));
}

q16_t VEC_length(const Vec2* vec){
    return Q16_sqrt(VEC_length_sq(vec));

}
q16_t VEC_dot(const Vec2* a,const Vec2* b){
    return a->x * b->x + a->y * b->y;
}
