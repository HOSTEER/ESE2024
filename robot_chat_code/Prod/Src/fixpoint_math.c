
#include "fixpoint_math.h"

/*static uint64_t frac16[16] = {5000000000000000, 2500000000000000, 1250000000000000, 625000000000000,
							  312500000000000, 156250000000000, 78125000000000, 39062500000000,
							  19531250000000, 9765625000000, 4882812500000, 2441406250000,
							  1220703125000, 610351562500, 305175781250, 152587890625};
*/

static uint64_t frac16[16] = {152587890625, 305175781250, 610351562500, 1220703125000,
							  2441406250000, 4882812500000, 9765625000000, 19531250000000,
							  39062500000000, 78125000000000, 156250000000000, 312500000000000,
							  625000000000000, 1250000000000000, 2500000000000000, 5000000000000000};

static int32_t atan2i[16] = {0xC90FDA, 0x76B19C,0x3EB6EB, 0x1FD5BA, 0xFFAAD, 0x7FF55, 0x3FFEA, 0x1FFFD,
							 0xFFFF,0x7FFF,0x3FFF,0x1FFF,0xFFF,0x7FF, 0x3FF, 0x1FF}; // arctan(2^-i) lookup table, Q7.24



/*uint64_t conv_frac16_dec(int16_t x, uint64_t scale)
{
	uint64_t out = 0;
	for(int i=0;i<16;i++)
	{
		out += ((x>>i)&1)*frac16[i];
	}
	return out/scale;
}*/

int32_t fixed_div_16(int32_t x, int32_t y)
{
	return (((int64_t)x) * (1 << 16)) / y;
}

int32_t fixed_mul_16(int32_t x, int32_t y)
{
    return ((int64_t)x * (int64_t)y) / (1 << 16);
}

int32_t fixed_mul(int32_t x, int32_t y, uint8_t qout)
{
	return ((int64_t)x * (int64_t)y) / (1 << qout);
}

int32_t fixed_div(int32_t x, int32_t y, uint8_t qout)
{
	return (((int64_t)x) * (1 << qout)) / y;
}


/**
  * @brief returns the sinus of angle
  * @param angle: angle from -2pi (-32768 to 32767) to 2pi, angle = rad/2pi
  *	@retval sinus(angle) value in Q3.12 int format
  */
int16_t fpsin(int16_t angle)
{
    /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which is the region of the curve fit). */
    /* ------------------------------------------------------------------- */
    angle <<= 1;
    uint8_t c = angle<0; //set carry for output pos/neg

    if(angle == (angle|0x4000)) // flip input value to corresponding value in range [0..8192)
        angle = (1<<15) - angle;
    angle = (angle & 0x7FFF) >> 1;
    /* ------------------------------------------------------------------- */

    /* The following section implements the formula:
     = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1 * y]) * 2^(a-q)
    Where the constants are defined as follows:
    */
    enum {A1=3370945099UL, B1=2746362156UL, C1=292421UL};
    enum {n=13, p=32, q=31, r=3, a=12};

    uint32_t y = (C1*((uint32_t)angle))>>n;
    y = B1 - (((uint32_t)angle*y)>>r);
    y = (uint32_t)angle * (y>>n);
    y = (uint32_t)angle * (y>>n);
    y = A1 - (y>>(p-q));
    y = (uint32_t)angle * (y>>n);
    y = (y+(1UL<<(q-a-1)))>>(q-a); // Rounding

    return c ? -y : y;
}

/**
  * @brief vector mode CORDIC, computes sqrt(x^2 + y^2) and Arctan(y/x)
  * @param *vector : address of vector structure containing x and y coordinates in Q15.16 format
  * 				 and fills the norm and angle inside vector structure
  */
void CORDIC_vector(vector_t *vector)
{
	uint8_t quadrant;
	uint32_t x_curr,x_next; //Q16.16
	int32_t y_curr; 	//Q15.16
	int32_t z_curr = 0; 	//Q15.16

	//shifts all coordinates in quadrant 1, x0 = |x|/2, y0 = |y|/2, z0 = 0
	if(vector->x >= 0)
	{
		x_curr = vector->x >> 1;
		if(vector->y >= 0)
		{
			quadrant = 1;
			y_curr = vector->y >> 1;
		}
		else
		{
			quadrant = 4;
			y_curr = -1*vector->y >> 1;
		}
	}
	else
	{
		x_curr = -1*vector->x >> 1;
		if(vector->y >= 0)
		{
			quadrant = 2;
			y_curr = vector->y >> 1;
		}
		else
		{
			quadrant = 3;
			y_curr = -1*vector->y >> 1;
		}
	}
	z_curr = 0;
	//printf("x_curr = %d, y_curr = %d, z_curr = %d\r\n", (int)x_curr,(int)y_curr,(int)z_curr);

	for(uint8_t i = 0; i < CORDIC_ITER; i++)
	{
		//xk+1 = xk + sign(yk)*yk/2
		//yk+1 = yk - sign(yk)*xk/2
		//zk+1 = z0 - sign(yk)*Arctan(2^-i)
		//
		//xn = K*sqrt(x0^2 + y0^2)
		//yn = 0
		//zn = z0 - Arctan(y0/x0)

		if(y_curr < 0) //3c
		{
			x_next = x_curr + (-1*y_curr >> i); //10c
			y_curr += x_curr >> i; //10c
			z_curr += atan2i[i]; //10c
		}
		else
		{
			x_next = x_curr + (y_curr >> i); //10c
			y_curr -= x_curr >> i; //10c
			z_curr -= atan2i[i]; //10c
		}
		x_curr = x_next;
		//printf("i = %d, x_curr = %d, y_curr = %d, z_curr = %d\r\n",(int)i, (int)x_curr, (int)y_curr, (int)z_curr);
	}

	vector->norm = (uint64_t)x_curr*CORDIC_CONSTANT >> 16; //Q16.16 30c

	switch(quadrant)
	{
		case 1:
			vector->angle = -1*z_curr;
			break;
		case 2:
			vector->angle = PI + z_curr;
			break;
		case 3:
			vector->angle = -1*(PI + z_curr);
			break;
		default: //case 4:
			vector->angle = z_curr;
			break;
	}


}



