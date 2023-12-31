
#include "main.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define ENC_ZERO_VAL 0
#define ENC_GAIN 0x26A2E14 //618.18 counts/rev, Q16.16
#define CURRENT_GAIN 0x27B7DA //39.71818181818182 inc/mA Q16.16
#define BATT_VOLTAGE_GAIN 0xC837119 //3203.4418 inc/V Q16.16

typedef struct hMotor_t_struct{
	//PWM and encoder timers
	TIM_HandleTypeDef *tim_FWD;
	TIM_HandleTypeDef *tim_REV;
	TIM_HandleTypeDef *tim_ENC;

	//adc DMA buffer and index
	volatile uint16_t * adc_dma_buff;
	uint16_t dma_buff_index;
	uint32_t current_offset;

	//Control parameters and buffers
	int32_t speed_output[3];
	int32_t speed_error[3];
	int32_t speed_measured[3];
	int8_t speed_anti_windup;
	int32_t speed_integral;
	uint32_t speed_corr_freq;
	uint32_t speed_corr_params[4]; //kp, ki, kd, sat
	uint8_t speed_index;

	int32_t current_measured;


}hMotor_t;

void current_sense_start();
uint32_t battery_get_voltage();
void motor_init(hMotor_t *hMotor, TIM_HandleTypeDef *tim_FWD, TIM_HandleTypeDef *tim_REV,
				TIM_HandleTypeDef *tim_ENC, uint8_t DMA_buff_index, uint32_t speed_kp,
				uint32_t speed_ki, uint32_t speed_kd, uint32_t sat, uint32_t speed_corr_freq);

void motor_set_PWM(hMotor_t *hMotor,int32_t speed);
void motor_get_speed(hMotor_t *hMotor);
void motor_get_current(hMotor_t *hMotor);

#endif /* INC_MOTOR_H_ */
