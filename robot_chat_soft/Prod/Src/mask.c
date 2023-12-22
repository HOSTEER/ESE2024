#include "mask.h"

/**
  * @brief Writes the minimum distance and the target angle in the target structure
  * @param *lidar : pointer to the lidar structure
  * @param *target : pointer to the target structure
  */
int find_target(h_ydlidar_x4_t * lidar, h_mask_target_t * target){
	// Searching the minimum distance
	uint16_t min_dist = MAX_TARGET_DIST, last_min = MAX_TARGET_DIST;
	for(int i=0 ; i<360 ; i++){
		if((lidar->sorted_dist[i] > 0) && (min_dist > lidar->sorted_dist[i])){
			min_dist = lidar->sorted_dist[i];
		}
		if(min_dist < last_min){
			target->target_angle = i;
			last_min = min_dist;
		}
	}

	// Identifying target
	target->target_shape[10] = min_dist;
	for(int k=1 ; k<11 ; k++){
		if(abs(lidar->sorted_dist[target->target_angle + k] - lidar->sorted_dist[target->target_angle + (k+1)]) <= TARGET_LIM){
			target->target_shape[10 + k] = lidar->sorted_dist[target->target_angle + k];
		}
		if(abs(lidar->sorted_dist[target->target_angle - k] - lidar->sorted_dist[target->target_angle - (k+1)]) <= TARGET_LIM){
			target->target_shape[10 - k] = lidar->sorted_dist[target->target_angle - k];
		}
	}
	return 0;
}

/**
  * @brief Approximate the distance of the target from the table center
  * @param *target : pointer to the target structure
  */
int target_dist_center(h_mask_target_t * target, hOdometry_t * odometry){
	int32_t xR = odometry->x;	// Robot coordinates
	int32_t yR = odometry->y;
	vector_t CR, RT, TC; 		// CR : Center->Robot, RT : Robot->Target, TC : Target->Center
	CR.x = xR - X_CENTER;
	CR.y = yR - Y_CENTER;
	//RT.x = fpcos(target->target_angle + odometry->angle);


}


