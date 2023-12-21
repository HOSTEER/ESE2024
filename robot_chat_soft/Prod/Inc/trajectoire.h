

#ifndef INC_TRAJECTOIRE_H_
#define INC_TRAJECTOIRE_H_

#include "main.h"
#include "odometry.h"

#define MAX_POS_ERROR (50<<16) //mm Q.16

typedef struct trajectory_t_struct {
	int32_t x,y;
	int32_t speed;
	int32_t angle_init,angle_end;

} trajectory_t;

int32_t follow_trajectory(hOdometry_t *hOdometry, int32_t *x, int32_t *y, int32_t *mot_speed);

#endif /* INC_TRAJECTOIRE_H_ */
