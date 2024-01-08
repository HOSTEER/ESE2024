/*
 * mask.h
 *
 *  Created on: Dec 20, 2023
 *      Author: mimib
 */

#ifndef INC_MASK_H_
#define INC_MASK_H_

#include "ydlidar_x4.h"
#include "fixpoint_math.h"
#include "odometry.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_TARGET_DIST 3600	// On the game table, furthest points are 3600mm apart
#define TARGET_LIM 50			// object limit if distance difference btwn 2 consecutive lidar points is > 5cm
#define X_CENTER (1500<<16)		// Q15.16
#define Y_CENTER (1000<<16)		// Q15.16

typedef struct h_mask_target_struct
{
	uint16_t angle;			// in °
	int32_t angle_rad;		// in radians
	uint16_t shape[21];		// identifying target shape on 20°
	uint16_t dist_min;
	uint16_t shape_ang_max;
	uint16_t shape_ang_min;
	int32_t dist_center;
} h_mask_target_t;


int find_target(h_ydlidar_x4_t * lidar, h_mask_target_t * target);
int target_dist_center(h_mask_target_t * target, hOdometry_t * odometry);

#endif /* INC_MASK_H_ */
