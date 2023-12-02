
#include "motor.h"

static volatile uint16_t adc_DMA_Buff[3] = {0};
extern ADC_HandleTypeDef hadc1;

void currentSenseStart()
{
	HAL_ADCEx_ChannelConfigReadyCallback(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_DMA_Buff, 3);
}

void motorInit(hMotor_t *hMotor, TIM_HandleTypeDef *tim_FWD, TIM_HandleTypeDef *tim_REV,
				TIM_HandleTypeDef *tim_ENC, uint8_t DMA_buff_index,
				uint32_t speed_kp, uint32_t speed_ki, uint32_t speed_kd, uint32_t sat,
				uint32_t current_kp, uint32_t current_ki, uint32_t current_kd)
{
	hMotor->tim_FWD = tim_FWD;
	hMotor->tim_REV = tim_REV;
	hMotor->tim_ENC = tim_ENC;

	HAL_TIM_PWM_Start(tim_FWD, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim_REV, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(tim_ENC, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	__HAL_TIM_SET_COUNTER(tim_ENC,32768);

	hMotor->adc_dma_buff = adc_DMA_Buff;
	hMotor->dma_buff_index = DMA_buff_index;

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

void motorGetSpeed(hMotor_t *hMotor)
{
	hMotor->speed_measured[hMotor->speed_index] = __HAL_TIM_GET_COUNTER(hMotor->tim_ENC) - 32768;
	__HAL_TIM_SET_COUNTER(hMotor->tim_ENC,32768);
}

void motorGetCurrent(hMotor_t *hMotor)
{
	hMotor->current_measured[hMotor->current_index] = hMotor->adc_dma_buff[hMotor->dma_buff_index];
}


