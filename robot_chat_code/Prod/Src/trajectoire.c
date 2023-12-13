
#include "trajectoire.h"


void follow_trajectory(hOdometry_t hOdometry, int32_t x, int32_t y, int32_t *angle, int32_t *mot_speed)
{
	vector_t trajectory_vector;
	trajectory_vector->x = x-hOdometry->x;
	trajectory_vector->y = y-hOdometry->y;

	CORDIC_vector(&trajectory_vector);
	if(trajectory_vector->norm < MAX_POS_ERROR)
	{
		*mot_speed = 0;
	}
	else
	{
		*angle = trajectory_vector->angle;
	}

}
