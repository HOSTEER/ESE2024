
#ifndef INC_FIXPOINT_MATH_H_
#define INC_FIXPOINT_MATH_H_

#include "main.h"

uint64_t conv_frac16_dec(uint16_t x, uint64_t scale);
int32_t fixed_div_16(int32_t x, int32_t y);
int32_t fixed_mul_16(int32_t x, int32_t y);
int32_t fixed_mul(int32_t x, int32_t y, uint8_t qout);
int32_t fixed_div(int32_t x, int32_t y, uint8_t qout);
int16_t fpsin(int16_t i);

//Cos(x) = sin(x + pi/2)
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))


#endif /* INC_FIXPOINT_MATH_H_ */
