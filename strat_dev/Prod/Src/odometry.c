
#include "odometry.h"
#include "fixpoint_math.h"
/*
void odometry_init(hOdometry_t *hOdometry, hMotor_t *Rmot, hMotor_t *Lmot, uint32_t diameter, uint32_t CPR, uint32_t wheel_dist, uint32_t freq)
{
	hOdometry->Rmot = Rmot;
	hOdometry->Lmot = Lmot;
	hOdometry->Cnt_dist_coeff = fixed_div(fixed_mul((int32_t)PI, (int32_t)diameter, 23),(int32_t)CPR,16); // mm/tick, Q8.24
	hOdometry->wheel_dist = wheel_dist; //mm, Q16.16
	hOdometry->freq = freq;
	hOdometry->dr = 0;
	hOdometry->angle = 0;
	hOdometry->x = 0;
	hOdometry->y = 0;
}*/
/*
void odometry_update_pos(hOdometry_t *hOdometry)
{
	hMotor_t *Rmot = hOdometry->Rmot;
	hMotor_t *Lmot = hOdometry->Lmot;

	//int16_t Rcount = (int16_t)__HAL_TIM_GET_COUNTER(Rmot->tim_ENC);
	//int16_t Lcount = (int16_t)__HAL_TIM_GET_COUNTER(Lmot->tim_ENC);

	//__HAL_TIM_SET_COUNTER(Rmot->tim_ENC, 0);
	//__HAL_TIM_SET_COUNTER(Lmot->tim_ENC, 0);


	//int32_t Rdelta = fixed_mul((int32_t)Rcount , hOdometry->Cnt_dist_coeff, 0); //mm Q8.24
	//int32_t Ldelta = -1*fixed_mul((int32_t)Lcount , hOdometry->Cnt_dist_coeff, 0); //mm Q8.24

	//Rmot->speed_measured[Rmot->speed_index] = fixed_mul(Rdelta, (int32_t)hOdometry->freq, 9) + Rmot->speed_measured[(Rmot->speed_index + 2)%3]/2; //right motor speed, mm/s, Q16.16
	//Lmot->speed_measured[Lmot->speed_index] = fixed_mul(Ldelta, (int32_t)hOdometry->freq, 9) + Lmot->speed_measured[(Lmot->speed_index + 2)%3]/2; //left motor speed, mm/s, Q16.16

	hOdometry->dr = Rdelta/2 - Ldelta/2; //mm, Q8.24
	int32_t dalpha = fixed_div(Rdelta, hOdometry->wheel_dist,16) + fixed_div(Ldelta, hOdometry->wheel_dist, 16); //rad, Q8.24

	int32_t dx = fixed_mul(hOdometry->dr, (int32_t)fpcos(fixed_div(hOdometry->angle + dalpha/2, PI<<2, 15)) * (1<<4), 24); //mm, Q16.16
	int32_t dy = fixed_mul(hOdometry->dr, (int32_t)fpsin(fixed_div(hOdometry->angle + dalpha/2, PI<<2, 15)) * (1<<4), 24); //mm, Q16.16

	hOdometry->angle += dalpha;


	if(hOdometry->angle > PI<<1)
	{
		hOdometry->angle = -(PI<<1) + hOdometry->angle%(PI<<1);
	}
	else if(hOdometry->angle < -(PI<<1))
	{
		hOdometry->angle = (PI<<1) - hOdometry->angle%(PI<<1);
	}

	hOdometry->x += dx;
	hOdometry->y += dy;

}*/
