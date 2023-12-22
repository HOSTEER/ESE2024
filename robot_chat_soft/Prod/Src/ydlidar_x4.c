#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "ydlidar_x4.h"
#include <string.h>
#include <stdlib.h>

#define FULL

int ydlidar_x4_init(h_ydlidar_x4_t * lidar){
	lidar->decode_state = SCANNING;
	lidar->serial_drv.receive(lidar->buf_DMA);
	lidar->nb_smpl = 0;
	lidar->start_angl = 0;
	lidar->end_angl = 0;
	memset(lidar->sorted_dist,10000,360);
	return 0;
}

int ydlidar_x4_stop(h_ydlidar_x4_t * lidar){
	lidar->cmd = CMD_STOP;
	lidar->serial_drv.transmit((uint8_t *) &(lidar->cmd), 2);
	return 0;
}

int ydlidar_x4_info(h_ydlidar_x4_t * lidar){
	lidar->cmd = CMD_INFO;
	lidar->serial_drv.transmit((uint8_t *) &(lidar->cmd), 2);
	return 0;
}

int ydlidar_x4_scan(h_ydlidar_x4_t * lidar){
	lidar->cmd = CMD_SCAN;
	lidar->serial_drv.transmit((uint8_t *) &(lidar->cmd), 2);
	return 0;
}

int ydlidar_x4_restart(h_ydlidar_x4_t * lidar){
	lidar->cmd = CMD_RESTART;
	lidar->serial_drv.transmit((uint8_t *) &(lidar->cmd), 2);
	return 0;
}

int ydlidar_x4_get_angle(h_ydlidar_x4_t * lidar, uint16_t angle_LSB, uint16_t angle_MSB){
	uint16_t * angle;
	if(lidar->decode_state == PARSING_START_ANGL){
		angle = &lidar->start_angl;
	}
	else{
		angle = &lidar->end_angl;
	}
	*angle 	= angle_LSB>>1;
	*angle 	= *angle + (angle_MSB<<7);
	*angle 	= *angle>>6;
	return 0;
}

int ydlidar_x4_get_dist(uint16_t * dist, uint16_t dist_LSB, uint16_t dist_MSB){
	*dist 	= dist_LSB;
	*dist 	= *dist + (dist_MSB<<8);
	*dist 	= *dist>>2;
	return 0;
}

int ydlidar_x4_store_smpl(h_ydlidar_x4_t * lidar){
	uint8_t smpl_idx=0;
	uint16_t angle_per_dist;
	uint16_t first_angle=lidar->start_angl;
	if(lidar->end_angl < first_angle){
		angle_per_dist = (uint16_t) ((lidar->end_angl + 360)-first_angle)>>2;
	}else{
		angle_per_dist = (uint16_t) (lidar->end_angl-first_angle)>>2;
	}
	for(;smpl_idx<40;smpl_idx++){
		if(lidar->smpl[smpl_idx] > 0){
			lidar->sorted_dist[(first_angle + (angle_per_dist*smpl_idx)/10)%359]= lidar->smpl[smpl_idx];
		}else{
			lidar->sorted_dist[(first_angle + (angle_per_dist*smpl_idx)/10)%359]= 10000;
		}
	}
	return 0;
}

int ydlidar_x4_irq_cb(h_ydlidar_x4_t * lidar){
	ydlidar_x4_parsing_t * state = &lidar->decode_state;
	uint8_t * dma_mem = lidar->buf_DMA;
	uint8_t * frame_smpl = &lidar->nb_smpl;
	uint8_t head_limit = 0;
	static uint8_t idx_head = 0;
	static uint8_t idx_limiter = 1;
	static uint8_t last_byte = 0;
	static uint8_t idx_filler = 0;

	if ( (idx_head >= (LIDAR2DMA_SIZE >> 1)) && (idx_head < LIDAR2DMA_SIZE) ) {
		head_limit = LIDAR2DMA_SIZE;
	}
	else {
		idx_head = 0;
		head_limit = (LIDAR2DMA_SIZE >> 1);

	}

	for(;idx_head < head_limit; last_byte = dma_mem[idx_head], idx_head++){
		switch(*state){
		case SCANNING :
			if(dma_mem[idx_head] == 0x55 && last_byte == 0xAA){
				*state = PARSING_SMPL;
				idx_limiter = 1;
			}
			break;
		case PARSING_SMPL :
			if(idx_limiter == 0){
				*frame_smpl = dma_mem[idx_head];
				if(*frame_smpl == 1){
					*state 		= SCANNING;
				}else{
					*state 		= PARSING_START_ANGL;
				}
				idx_limiter = 1;
			}
			else{
				idx_limiter --;
			}
			break;

		case PARSING_START_ANGL :
			if(idx_limiter == 0){
				ydlidar_x4_get_angle(lidar, (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
				*state = PARSING_END_ANGL;
				idx_limiter = 1;
			}
			else{
				idx_limiter --;
			}
			break;

		case PARSING_END_ANGL :
			if(idx_limiter == 0){
				ydlidar_x4_get_angle(lidar, (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
				*state = PARSING_DIST;
				idx_limiter = 0;
			}
			else{
				idx_limiter --;
			}
			break;
		case PARSING_DIST :
			if(((idx_limiter%2) != 0) && (idx_limiter < *frame_smpl)){
				ydlidar_x4_get_dist(&lidar->smpl[idx_filler], (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
				idx_filler++;
				idx_limiter ++;
			}
			else{
				idx_limiter ++;
				if(idx_limiter > *frame_smpl){
					idx_filler = 0;
					ydlidar_x4_store_smpl(lidar);

					memset(lidar->smpl,0,(*frame_smpl)*2);
					lidar->start_angl=0;
					lidar->end_angl=0;
					*frame_smpl = 0;
					*state = SCANNING;

				}
			}
			break;
		}
	}
	return 0;
}
