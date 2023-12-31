

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_
#include "motor.h"



typedef struct hOdometry_t_struct {
	hMotor_t *Rmot; //right motor
	hMotor_t *Lmot; //left motor

	uint32_t Cnt_dist_coeff; //encoder tick to mm conversion, 2pi*wheel_radius/ticks_per_rev, mm/tick, Q8.24
	uint32_t wheel_dist; //distance between wheels, mm, Q16.16
	uint32_t freq;	//refresh rate, Hz
	int32_t dr;
	int32_t x,y; //robot coordinates Q16.16

	int32_t angle; //robot angle Q8.24 [-pi, pi]

} hOdometry_t;


void odometry_init(hOdometry_t *hOdometry, hMotor_t *Rmot, hMotor_t *Lmot, uint32_t radius, uint32_t CPR, uint32_t wheel_dist, uint32_t freq);
void odometry_update_pos(hOdometry_t *hOdometry);

#endif /* INC_ODOMETRY_H_ */
