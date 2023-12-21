/*
 * mask.h
 *
 *  Created on: Dec 20, 2023
 *      Author: mimib
 */

#ifndef INC_MASK_H_
#define INC_MASK_H_

#include "ydlidar_x4.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_TARGET_DIST 3600		// On the game table, furthest points are 3600mm apart
#define TARGET_LIM 50			// object limit if distance difference btwn 2 consecutive lidar points is > 5cm

typedef struct h_mask_target_struct
{
	uint16_t target_angle;
	uint16_t target_shape[21];	// identifying target shape on 20°
} h_mask_target_t;


int find_target(h_ydlidar_x4_t * lidar, h_mask_target_t * target);

#endif /* INC_MASK_H_ */
