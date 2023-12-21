
#include "control.h"
#include "fixpoint_math.h"

/**
  * @brief Motor speed control loop, this function should be called right after odometry_update_pos()
  * @param *hMotor : address of motor structure containing motor parameters.
  *	@retval None
  */
void set_speed_PID(hMotor_t * hMotor,int32_t input)
{
	hMotor->speed_error[hMotor->speed_index] = input - hMotor->speed_measured[hMotor->speed_index];

	hMotor->speed_integral += hMotor->speed_anti_windup*(hMotor->speed_error[(hMotor->speed_index + 2)%3] + hMotor->speed_error[hMotor->speed_index]);

	hMotor->speed_anti_windup = 1;

	hMotor->speed_output[hMotor->speed_index] = fixed_mul_16(hMotor->speed_error[hMotor->speed_index], (int32_t)hMotor->speed_corr_params[0]) +
												fixed_mul_16(hMotor->speed_integral, (int32_t)hMotor->speed_corr_params[1]) +
												fixed_mul_16(hMotor->speed_measured[(hMotor->speed_index + 2)%3] - hMotor->speed_measured[hMotor->speed_index], (int32_t)hMotor->speed_corr_params[2]) +
												fixed_mul(input,5<<12,16) /*hMotor->speed_output[(hMotor->speed_index + 2)%3] /*+
												fixed_mul_16(hMotor->speed_error[(hMotor->speed_index + 2)%3], (hMotor->speed_corr_params[1] - hMotor->speed_corr_params[2])*2) +
												fixed_mul_16(hMotor->speed_error[(hMotor->speed_index + 1)%3], hMotor->speed_corr_params[1] + hMotor->speed_corr_params[2] - hMotor->speed_corr_params[0]) +
												hMotor->speed_output[(hMotor->speed_index - 2)%3]*/;

	//Current saturation based on the difference between drive voltage and BEMF,
	//prevents OCD triggering, which can cause a motor to stall and never ramp up again

	if(fixed_div(hMotor->speed_output[hMotor->speed_index],128<<16,16) - fixed_div(hMotor->speed_measured[hMotor->speed_index],80<<16,16) > 1<<18)
	{
		hMotor->speed_output[hMotor->speed_index] = fixed_mul(fixed_div(hMotor->speed_measured[hMotor->speed_index],80<<16,16) + (1<<18), 128<<16, 16);
		hMotor->speed_anti_windup = 0;
	}
	else if(fixed_div(hMotor->speed_output[hMotor->speed_index],128<<16,16) - fixed_div(hMotor->speed_measured[hMotor->speed_index],80<<16,16) < -1<<18)
	{
		hMotor->speed_output[hMotor->speed_index] = fixed_mul(fixed_div(hMotor->speed_measured[hMotor->speed_index],80<<16,16) - (1<<18), 128<<16, 16);
		hMotor->speed_anti_windup = 0;
	}

	//PWM saturation

	if(hMotor->speed_output[hMotor->speed_index] > (int32_t)hMotor->speed_corr_params[3])
	{
		hMotor->speed_output[hMotor->speed_index] = (int32_t)hMotor->speed_corr_params[3];
		hMotor->speed_anti_windup = 0;

	}
	else if(hMotor->speed_output[hMotor->speed_index] < -1*((int32_t)hMotor->speed_corr_params[3]))
	{
		hMotor->speed_output[hMotor->speed_index] = -1*((int32_t)hMotor->speed_corr_params[3]);
		hMotor->speed_anti_windup = 0;
	}
	motor_set_PWM(hMotor, hMotor->speed_output[hMotor->speed_index]/(1<<16));
	hMotor->speed_index = (hMotor->speed_index + 1) % 3;
	//TODO fully implement feedforward
}

int32_t error_old = 0;
int32_t integral = 0;
int32_t anti_windup = 1;
/**
  * @brief Angular position control loop, this function should be called regularly.
  * TODO implement full corrector
  * @param *hMotor : address of motor structure containing motor parameters.
  *	@retval diff : return L/R speed coefficient
  */
int32_t set_angle_corr(hOdometry_t * hOdometry, int32_t input)
{
	int32_t output = 0; //Q7.24
	int32_t error = modulo_2pi(input - hOdometry->angle); //Q7.24
	integral += anti_windup*error/10;
	anti_windup = 1;
	//printf("a_err = %d\r\n",10*(int)hOdometry->angle/(1<<24));

	output = 5*error - 15*(hOdometry->angle - error_old);
	error_old = hOdometry->angle;

	if(output > 1<<23)
	{
		anti_windup = 0;
	}
	else if(output < -1<<23)
	{
		anti_windup = 0;
	}
	return fixed_mul(Arctan(output/(1<<8)),INV_PI,23);
	//return output;
}

