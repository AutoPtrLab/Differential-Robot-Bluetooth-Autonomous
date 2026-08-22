// sin and cos
// take into account that we are working with rad * 1000000
#ifndef TRIGONOMETRY_HELPER_H
#define TRIGONOMETRY_HELPER_H
#include <stdint.h>

// maybe making this instead of being in 10 exp(5) be in 2exp(*14) so is more
// optimazed
extern const int16_t look_up_table[361];

#define URAD_PER_DEGREE 17453  // microrad per degree
#define URAD_90_DEGREE 1570796 // 90 degrees in Urad
#define TWO_PI_URAD 6283185    // 360 degrees in microrads

// since we cant store all the posible decimals in the look up table we are
// going to asume that in the proximitise the sin and cosine functions behave
// like a linear function and can be interpolate

// this funciton receives a micro rad and return  the sine of it in x10exp(5) ,
// THE ANG MUST BE PRENORMALIZED BETWEEN 0 and 2PI * 10^6
static int16_t urad_sin(int32_t ang) {
    // normalize the angle
    while (ang >= TWO_PI_URAD)
        ang -= TWO_PI_URAD;
    while (ang < 0)
        ang += TWO_PI_URAD;
    // we get the two frontier values
    unsigned int base_ang = ang / URAD_PER_DEGREE;
    int16_t rest_ang = ang % 100;
    // nearest right side value
    int16_t y1 = look_up_table[base_ang];
    int16_t y2 = look_up_table[base_ang + 1];
    // interpolation result = y1 + ((y2- y1)/(x2-x1)) * (x-x1)
    // x2-x1 its always the distance between the two nearest readable angles
    int16_t result = y1 + ((int32_t)((y2 - y1) * rest_ang) /
                           URAD_PER_DEGREE); // need of the int32 conversion to dodge the overflow
    // linear interpolation
    return result;
}
// this funciton receives a micro rad and resturns the cosine of it in x10exp(5)
// , THE ANG MUST BE PRENORMALIZED BETWEEN 0 and 2PI * 10^6
static int16_t urad_cos(uint32_t ang) {
    return urad_sin(ang + URAD_90_DEGREE); // since the cosine is equal to the sine function
                                           // but with a phase lag of 90n  degrees
}

#endif
