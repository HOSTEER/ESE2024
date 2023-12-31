
#include "motor.h"
#include "fixpoint_math.h"

static volatile uint16_t adc_DMA_Buff[3] = {0};
extern ADC_HandleTypeDef hadc1;


/**
  * @brief starts motor current and battery voltage sensing ADC
  */
void current_sense_start()
{
	HAL_ADCEx_ChannelConfigReadyCallback(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_DMA_Buff, 3);
}

/**
  * @brief extracts sampled value from ADC's DMA, converts it to a voltage
  * @retval voltage : battery voltage, Q16.16
  */
uint32_t battery_get_voltage()
{
	return fixed_div_16(((uint32_t)adc_DMA_Buff[0])<<15, BATT_VOLTAGE_GAIN);
}

/**
  * @brief configures motor structure with input parameters
  * @param *hMotor : motor structure address to be configured
  * @param *tim_FWD : forward half-bridge PWM timer structure address
  * @param *tim_REV : reverse half-bridge PWM timer structure address
  * @param *tim_ENC : encoder timer structure address
  * @param DMA_buff_index : ADC's DMA buffer position for current sensing
  * @param speed_kp : proportionnal corrector coefficient
  * @param speed_ki : integral corrector coefficient
  * @param speed_ki : derivative corrector coefficient
  * @param sat : PWM saturation, set to pwm resolution
  * @param speed_corr_freq : corrector frequency
  * @retval None
  */
void motor_init(hMotor_t *hMotor, TIM_HandleTypeDef *tim_FWD, TIM_HandleTypeDef *tim_REV,
				TIM_HandleTypeDef *tim_ENC, uint8_t DMA_buff_index, uint32_t speed_kp,
				uint32_t speed_ki, uint32_t speed_kd, uint32_t sat, uint32_t speed_corr_freq)
{
	hMotor->tim_FWD = tim_FWD;
	hMotor->tim_REV = tim_REV;
	hMotor->tim_ENC = tim_ENC;

	HAL_TIM_PWM_Start(tim_FWD, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim_REV, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(tim_ENC, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	__HAL_TIM_SET_COUNTER(tim_ENC,ENC_ZERO_VAL);

	hMotor->adc_dma_buff = adc_DMA_Buff;
	hMotor->dma_buff_index = DMA_buff_index;
	hMotor->current_offset = 0;
	hMotor->current_measured = 0;

	hMotor->speed_corr_freq = speed_corr_freq;
	hMotor->speed_integral = 0;
	hMotor->speed_anti_windup = 0;

	for(int i=0; i<3;i++)
	{
		hMotor->speed_output[i] = 0;
		hMotor->speed_measured[i] = 0;
		hMotor->speed_error[i] = 0;
	}

	hMotor->speed_corr_params[0] = speed_kp;
	hMotor->speed_corr_params[1] = speed_ki;
	hMotor->speed_corr_params[2] = speed_kd;
	hMotor->speed_corr_params[3] = sat;
}

/**
  * @brief set motor PWM
  * @param *hMotor : motor structure address
  * @param speed : PWM value, set a negative value to turn in reverse
  * @retval None
  */
void motor_set_PWM(hMotor_t *hMotor,int32_t speed)
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

/**
  * @brief store motor speed value from encoder timer, resets counter value
  * 	   DO NOT USE outside of odometry_update_pos()
  * @param *hMotor : motor structure address
  * @retval None
  */
void motor_get_speed(hMotor_t *hMotor)
{
	hMotor->speed_measured[hMotor->speed_index] = fixed_div_16(((int16_t)(__HAL_TIM_GET_COUNTER(hMotor->tim_ENC) - ENC_ZERO_VAL))*(1<<16),ENC_GAIN);
	__HAL_TIM_SET_COUNTER(hMotor->tim_ENC, ENC_ZERO_VAL);
}

/**
  * @brief store motor current value from ADC
  * @param *hMotor : motor structure address
  * @retval None
  */
void motor_get_current(hMotor_t *hMotor)
{
	hMotor->current_measured = fixed_div_16(((int32_t)(hMotor->adc_dma_buff[hMotor->dma_buff_index] - hMotor->current_offset))*(1<<16),CURRENT_GAIN);
}

