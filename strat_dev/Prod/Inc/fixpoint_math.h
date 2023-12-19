
#ifndef INC_FIXPOINT_MATH_H_
#define INC_FIXPOINT_MATH_H_

#include "main.h"

#define CORDIC_ITER 8 //number of iterations of the CORDIC algorithm, max = 16 (limited by atan2i table), ~50c/iter
#define CORDIC_CONSTANT 0x136E9 //multiplicative constant to get accurate norm, Q.16.16

#define PI 0x3243F6A //Pi, Q8.24
#define HALF_PI (0x3243F6A>>1)

typedef struct vector_t_struct {
	int32_t x, y, angle, norm;
} vector_t;

//uint64_t conv_frac16_dec(uint16_t x, uint64_t scale);
void CORDIC_vector(vector_t *vector);
int32_t fixed_div_16(int32_t x, int32_t y);
int32_t fixed_mul_16(int32_t x, int32_t y);
int32_t fixed_mul(int32_t x, int32_t y, uint8_t qout);
int32_t fixed_div(int32_t x, int32_t y, uint8_t qout);
int16_t fpsin(int16_t i);
//Cos(x) = sin(x + pi/2)
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))


#endif /* INC_FIXPOINT_MATH_H_ */
