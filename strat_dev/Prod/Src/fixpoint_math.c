
#include <stdint.h>

/*static uint64_t frac16[16] = {5000000000000000, 2500000000000000, 1250000000000000, 625000000000000,
							  312500000000000, 156250000000000, 78125000000000, 39062500000000,
							  19531250000000, 9765625000000, 4882812500000, 2441406250000,
							  1220703125000, 610351562500, 305175781250, 152587890625};
*/

static uint64_t frac16[16] = {152587890625, 305175781250, 610351562500, 1220703125000,
							  2441406250000, 4882812500000, 9765625000000, 19531250000000,
							  39062500000000, 78125000000000, 156250000000000, 312500000000000,
							  625000000000000, 1250000000000000, 2500000000000000, 5000000000000000};

uint64_t conv_frac16_dec(int16_t x, uint64_t scale)
{
	uint64_t out = 0;
	for(int i=0;i<16;i++)
	{
		out += ((x>>i)&1)*frac16[i];
	}
	return out/scale;
}

int32_t fixed_div_16(int32_t x, int32_t y)
{
	int32_t quot;
	if(x >= 0){
		return (((int64_t)x) * (1 << 16)) / y;
	}
	else{
		x = -x;
		quot = (((int64_t)x) * (1 << 16)) / y;
		return -quot;
	}
}

int32_t fixed_mul_16(int32_t x, int32_t y)
{
	int32_t prod;
	prod = (int64_t)x * (int64_t)y;
	if(prod >= 0){
		return prod / (1 << 16);
	}
	else{
		prod = -prod;
		prod = prod / (1 << 16);
		return -prod;
	}
    //return ((int64_t)x * (int64_t)y) / (1 << 16);
}

int32_t fixed_mul(int32_t x, int32_t y, uint8_t qout)
{
	return ((int64_t)x * (int64_t)y) / (1 << qout);
}

int32_t fixed_div(int32_t x, int32_t y, uint8_t qout)
{
	return (((int64_t)x) * (1 << qout)) / y;
}

int16_t fpsin(int16_t i)
{
    /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which is the region of the curve fit). */
    /* ------------------------------------------------------------------- */
    i <<= 1;
    uint8_t c = i<0; //set carry for output pos/neg

    if(i == (i|0x4000)) // flip input value to corresponding value in range [0..8192)
        i = (1<<15) - i;
    i = (i & 0x7FFF) >> 1;
    /* ------------------------------------------------------------------- */

    /* The following section implements the formula:
     = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1 * y]) * 2^(a-q)
    Where the constants are defined as follows:
    */
    enum {A1=3370945099UL, B1=2746362156UL, C1=292421UL};
    enum {n=13, p=32, q=31, r=3, a=12};

    uint32_t y = (C1*((uint32_t)i))>>n;
    y = B1 - (((uint32_t)i*y)>>r);
    y = (uint32_t)i * (y>>n);
    y = (uint32_t)i * (y>>n);
    y = A1 - (y>>(p-q));
    y = (uint32_t)i * (y>>n);
    y = (y+(1UL<<(q-a-1)))>>(q-a); // Rounding

    return c ? -y : y;
}
