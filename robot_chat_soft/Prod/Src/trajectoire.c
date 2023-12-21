
#include "trajectoire.h"
#include "odometry.h"
#include "fixpoint_math.h"


int32_t follow_trajectory(hOdometry_t *hOdometry, int32_t *x, int32_t *y, int32_t *mot_speed)
{
	vector_t trajectory_vector;
	vector_t test_vector;
	trajectory_vector.x = *x-hOdometry->x;
	trajectory_vector.y = *y-hOdometry->y;

	CORDIC_vector(&trajectory_vector);
	//printf("angle = %d\r\n",10*(int)(trajectory_vector.angle)/(1<<24));
	if(trajectory_vector.norm < MAX_POS_ERROR)
	{
		test_vector.x = (1000<<16)-hOdometry->x;
		test_vector.y = hOdometry->y;
		CORDIC_vector(&test_vector);
		if(test_vector.norm < MAX_POS_ERROR)
		{
			*y = 1000<<16;
		}
		else
		{
			test_vector.x = (1000<<16) - hOdometry->x;
			test_vector.y = (1000<<16) - hOdometry->y;
			CORDIC_vector(&test_vector);
			if(test_vector.norm < MAX_POS_ERROR)
			{
				*x = 0;
			}
			else
			{
				test_vector.x = hOdometry->x;
				test_vector.y = (1000<<16) - hOdometry->y;
				CORDIC_vector(&test_vector);
				if(test_vector.norm < MAX_POS_ERROR)
				{
					*y = 0;
				}
				else
				{
					test_vector.x = hOdometry->x;
					test_vector.y = hOdometry->y;
					CORDIC_vector(&test_vector);
					if(test_vector.norm < MAX_POS_ERROR)
					{
						*x = 1000<<16;
					}
				}
			}
		}
	}
	return trajectory_vector.angle;

}
