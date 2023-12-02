/*
 * motor.h
 *
 *  Created on: Dec 2, 2023
 *      Author:
 */

#include "main.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_



typedef struct hMotor_t_struct{
	//PWM and encoder timers
	TIM_HandleTypeDef *tim_FWD;
	TIM_HandleTypeDef *tim_REV;
	TIM_HandleTypeDef *tim_ENC;

	//current sense ADC
	ADC_HandleTypeDef *adc_current;
	uint32_t adc_channel;

	//Control parameters and buffers
	int32_t speed_output[3];
	int32_t speed_error[3];
	int32_t speed_measured[3];
	uint32_t speed_corr_params[4]; //kp, ki, kd, sat
	uint8_t speed_index;

	int32_t current_output[3];
	int32_t current_error[3];
	int32_t current_measured[3];
	uint32_t current_corr_params[3]; //kp, ki, kd
	uint8_t current_index;


}hMotor_t;


void motorSetSpeed(hMotor_t *hMotor,int32_t speed);
void motorInit(hMotor_t *hMotor, TIM_HandleTypeDef *tim_FWD, TIM_HandleTypeDef *tim_REV,
				TIM_HandleTypeDef *tim_ENC, ADC_HandleTypeDef *adc_current, uint32_t adc_channel,
				uint32_t speed_kp, uint32_t speed_ki, uint32_t speed_kd, uint32_t sat,
				uint32_t current_kp, uint32_t current_ki, uint32_t current_kd);
#endif /* INC_MOTOR_H_ */
