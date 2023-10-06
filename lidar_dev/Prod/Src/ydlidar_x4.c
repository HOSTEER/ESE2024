#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "ydlidar_x4.h"
#include <string.h>
#include <stdlib.h>

int ylidar_x4_stop(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_STOP;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) &(h_ylidar_x4->cmd), 2);
	return 0;
}

int ylidar_x4_info(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_INFO;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) &(h_ylidar_x4->cmd), 2);
	return 0;
}

int ylidar_x4_scan(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_SCAN;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) &(h_ylidar_x4->cmd), 2);
	return 0;
}

int ylidar_x4_restart(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_RESTART;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) &(h_ylidar_x4->cmd), 2);
	return 0;
}

int ylidar_x4_get_angle(h_ylidar_x4_t * h_ylidar_x4, uint16_t angle_LSB, uint16_t angle_MSB){
	uint16_t * angle;
	if(h_ylidar_x4->decode_state == PARSING_START_ANGL){
		angle = &h_ylidar_x4->start_angl;
	}
	else{
		angle = &h_ylidar_x4->end_angl;
	}
	*angle 	= angle_LSB>>1;
	*angle 	= *angle + (angle_MSB<<7);
	*angle 	= *angle>>6;
	return 0;
}

int ylidar_x4_get_dist(uint16_t * dist, uint16_t dist_LSB, uint16_t dist_MSB){
	*dist 	= dist_LSB;
	*dist 	= *dist + (dist_MSB<<8);
	*dist 	= *dist>>2;
	return 0;
}

int ylidar_x4_store_smpl(h_ylidar_x4_t * h_ylidar_x4){
	uint8_t smpl_idx=0;
	static uint16_t revoltion_idx=0;
	uint16_t angle_per_dist = (uint16_t) abs(h_ylidar_x4->end_angl-h_ylidar_x4->start_angl)/4;
	uint16_t first_angle=h_ylidar_x4->start_angl;
	for(;smpl_idx<40;smpl_idx++){
		if(h_ylidar_x4->smpl[smpl_idx] != 0){
			h_ylidar_x4->rev_smpls[revoltion_idx][0]=first_angle + (angle_per_dist*smpl_idx)/10;
			h_ylidar_x4->rev_smpls[revoltion_idx][1]=h_ylidar_x4->smpl[smpl_idx];
			revoltion_idx++;
		}
	}
	if (revoltion_idx>600) {
		revoltion_idx = 0;
	}
	return 0;
}

int ydlidar_x4_irq_cb(h_ylidar_x4_t * h_ylidar_x4){
	uint8_t * dma_mem = h_ylidar_x4->buf_DMA;
	uint8_t * frame_smpl = &h_ylidar_x4->nb_smpl;
	uint8_t dma_size = LIDAR2DMA_SIZE;
	ylidar_x4_parsing_t * state = &h_ylidar_x4->decode_state;
	uint8_t head_limit = 0;
	static uint8_t idx_head = 0;
	static uint8_t idx_limiter = 1;
	if ( (idx_head > (dma_size >> 1)) && (idx_head < dma_size) ) {
		head_limit = dma_size;
	}
	else {
		idx_head = 0;
		head_limit = (dma_size >> 1);

	}
	static uint8_t last_byte = 0;
	while(idx_head < head_limit){
		if(*state == IDLE){
			if(dma_mem[0] == 0xA5 && dma_mem[1] == 0x5A){
					*state = SCANNING;
					idx_head = 26;
			}
			else{
				return -1;
			}
		}
		else if(*state == SCANNING){
			if(dma_mem[idx_head] == 0x55 && last_byte == 0xAA){
				*state = PARSING_SMPL;
				idx_limiter = 1;
			}
		}
		else if(*state == PARSING_SMPL){
			if(idx_limiter == 0){
				*frame_smpl = dma_mem[idx_head];
				*state 		= PARSING_START_ANGL;
				idx_limiter = 1;
			}
			else{
				idx_limiter --;
			}
		}
		else if(*state == PARSING_START_ANGL){
			if(idx_limiter == 0){
				ylidar_x4_get_angle(h_ylidar_x4, (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
				*state = PARSING_END_ANGL;
				idx_limiter = 1;
			}
			else{
				idx_limiter --;
			}
		}
		else if(*state == PARSING_END_ANGL){
			if(idx_limiter == 0){
				ylidar_x4_get_angle(h_ylidar_x4, (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
				*state = PARSING_DIST;
				idx_limiter = 0;
			}
			else{
				idx_limiter --;
			}
		}
		else if(*state == PARSING_DIST){
			static uint8_t idx_filler = 0;
			if(((idx_limiter%2) != 0) && (idx_limiter < *frame_smpl)){
				ylidar_x4_get_dist(&h_ylidar_x4->smpl[idx_filler], (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
				idx_filler++;
				idx_limiter ++;
			}
			else{
				idx_limiter ++;
				if(idx_limiter > *frame_smpl){
					idx_filler = 0;
					ylidar_x4_store_smpl(h_ylidar_x4);
					memset(h_ylidar_x4->smpl,0,(*frame_smpl)*2);
					h_ylidar_x4->start_angl=0;
					h_ylidar_x4->end_angl=0;
					*frame_smpl = 0;
					*state = SCANNING;
				}
			}
		}
		last_byte = dma_mem[idx_head];
		idx_head++;
	}
	return 0;
}
