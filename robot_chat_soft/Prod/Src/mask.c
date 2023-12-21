#include "mask.h"


int find_target(h_ydlidar_x4_t * lidar, h_mask_target_t * target){
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


