
#include "main.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_



typedef struct hMotor_t_struct{
	//PWM and encoder timers
	TIM_HandleTypeDef *tim_FWD;
	TIM_HandleTypeDef *tim_REV;
	TIM_HandleTypeDef *tim_ENC;

	//adc DMA buffer and index
	uint16_t * adc_dma_buff;
	uint16_t dma_buff_index;

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

void currentSenseStart();
void motorInit(hMotor_t *hMotor, TIM_HandleTypeDef *tim_FWD, TIM_HandleTypeDef *tim_REV,
				TIM_HandleTypeDef *tim_ENC, uint8_t DMA_buff_index,
				uint32_t speed_kp, uint32_t speed_ki, uint32_t speed_kd, uint32_t sat,
				uint32_t current_kp, uint32_t current_ki, uint32_t current_kd);

void motorSetSpeed(hMotor_t *hMotor,int32_t speed);
void motorGetSpeed(hMotor_t *hMotor);
void motorGetCurrent(hMotor_t *hMotor);

#endif /* INC_MOTOR_H_ */
