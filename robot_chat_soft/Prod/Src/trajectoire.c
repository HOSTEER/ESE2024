
#include "trajectoire.h"
#include "odometry.h"
#include "fixpoint_math.h"


int32_t follow_trajectory(hOdometry_t *hOdometry, int32_t x, int32_t y, int32_t *mot_speed)
{
	vector_t trajectory_vector;
	trajectory_vector.x = x-hOdometry->x;
	trajectory_vector.y = y-hOdometry->y;

	CORDIC_vector(&trajectory_vector);
	//printf("angle = %d\r\n",10*(int)(trajectory_vector.angle)/(1<<24));
	if(trajectory_vector.norm < MAX_POS_ERROR)
	{
		*mot_speed = 0;
		printf("stop\r\n");
	}
	return trajectory_vector.angle;

}
