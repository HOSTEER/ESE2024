/*
 * motor.c
 *
 *  Created on: Dec 2, 2023
 *      Author:
 */

#include "motor.h"

void motorInit(hMotor_t *hMotor, TIM_HandleTypeDef *tim_FWD, TIM_HandleTypeDef *tim_REV,
				TIM_HandleTypeDef *tim_ENC, ADC_HandleTypeDef *adc_current, uint32_t adc_channel,
				uint32_t speed_kp, uint32_t speed_ki, uint32_t speed_kd, uint32_t sat,
				uint32_t current_kp, uint32_t current_ki, uint32_t current_kd)
{
	hMotor->tim_FWD = tim_FWD;
	hMotor->tim_REV = tim_REV;
	hMotor->tim_ENC = tim_ENC;

	hMotor->adc_current = adc_current;
	hMotor->adc_channel = adc_channel;

	for(int i=0; i<3;i++)
	{
		hMotor->speed_output[i] = 0;
		hMotor->speed_measured[i] = 0;
		hMotor->speed_error[i] = 0;

		hMotor->current_output[i] = 0;
		hMotor->current_measured[i] = 0;
		hMotor->current_error[i] = 0;
	}

	hMotor->speed_corr_params[0] = speed_kp;
	hMotor->speed_corr_params[1] = speed_ki;
	hMotor->speed_corr_params[2] = speed_kd;
	hMotor->speed_corr_params[3] = sat;

	hMotor->current_corr_params[0] = current_kp;
	hMotor->current_corr_params[1] = current_ki;
	hMotor->current_corr_params[2] = current_kd;
}

void motorSetSpeed(hMotor_t *hMotor,int32_t speed)
{
	if(speed > 0)
	{
		//FWD PWM mode, FWD => PWM, REV => L;
		__HAL_TIM_SET_COMPARE(hMotor->tim_FWD,TIM_CHANNEL_1,speed);
		__HAL_TIM_SET_COMPARE(hMotor->tim_REV,TIM_CHANNEL_1,0);
	}
	else
	{
		//REV PWM mode, FWD => L, REV => PWM;
		__HAL_TIM_SET_COMPARE(hMotor->tim_FWD,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(hMotor->tim_REV,TIM_CHANNEL_1,-1*speed);
	}
}


