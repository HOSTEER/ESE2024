
#include "control.h"
#include "fixpoint_math.h"


void set_speed_PID(hMotor_t * hMotor,int32_t input)
{
	hMotor->speed_error[hMotor->speed_index] = input - hMotor->speed_measured[hMotor->speed_index];

	hMotor->speed_integral += hMotor->speed_anti_windup*(hMotor->speed_error[(hMotor->speed_index + 2)%3] + hMotor->speed_error[hMotor->speed_index]);

	hMotor->speed_anti_windup = 1;

	hMotor->speed_output[hMotor->speed_index] = fixed_mul_16(hMotor->speed_error[hMotor->speed_index], (int32_t)hMotor->speed_corr_params[0]) +
												fixed_mul_16(hMotor->speed_integral, (int32_t)hMotor->speed_corr_params[1]) +
												fixed_mul_16(hMotor->speed_measured[(hMotor->speed_index + 2)%3] - hMotor->speed_measured[hMotor->speed_index], (int32_t)hMotor->current_corr_params[2]) +
												fixed_mul(input,5<<14,16) /*hMotor->speed_output[(hMotor->speed_index + 2)%3] /*+
												fixed_mul_16(hMotor->speed_error[(hMotor->speed_index + 2)%3], (hMotor->speed_corr_params[1] - hMotor->speed_corr_params[2])*2) +
												fixed_mul_16(hMotor->speed_error[(hMotor->speed_index + 1)%3], hMotor->speed_corr_params[1] + hMotor->speed_corr_params[2] - hMotor->speed_corr_params[0]) +
												hMotor->speed_output[(hMotor->speed_index - 2)%3]*/;
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

