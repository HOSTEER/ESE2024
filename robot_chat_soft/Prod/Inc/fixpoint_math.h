
#ifndef INC_FIXPOINT_MATH_H_
#define INC_FIXPOINT_MATH_H_

#include "main.h"

#define CORDIC_ITER 8 //number of iterations of the CORDIC algorithm, max = 16 (limited by atan2i table), ~50c/iter
#define CORDIC_CONSTANT 0x136E9 //multiplicative constant to get accurate norm, Q.16

#define TWO_PI 0x6487ED4 //2pi, Q.24
#define PI 0x3243F6A //pi, Q.24
#define HALF_PI 0x1921FB5 //pi/2, Q.24
#define INV_PI 0x517CC2 // 1/pi, Q.24
#define DEG2RAD 0x4773D // 180/pi, Q.24

//vector structure, containing coordinates in x,y Q15.16 format and/or in norm, angle Q15.16, Q7.24.
typedef struct vector_t_struct {
	int32_t x, y, angle, norm;
} vector_t;

//uint64_t conv_frac16_dec(uint16_t x, uint64_t scale);
void CORDIC_vector(vector_t *vector);

int32_t fixed_div_16(int32_t x, int32_t y);

int32_t fixed_mul_16(int32_t x, int32_t y);

int32_t fixed_mul(int32_t x, int32_t y, uint8_t rs);

int32_t fixed_div(int32_t x, int32_t y, uint8_t ls);

int32_t fpsin(int32_t angle, uint8_t qout);

int32_t modulo_2pi(int32_t angle);

int32_t fpcos(int32_t angle, uint8_t qout);

//Cos(x) = sin(x + pi/2)



#endif /* INC_FIXPOINT_MATH_H_ */
