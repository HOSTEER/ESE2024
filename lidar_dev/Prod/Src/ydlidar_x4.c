#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "ydlidar_x4.h"
#include <string.h>
#include <stdlib.h>

int ydlidar_x4_stop(h_ydlidar_x4_t * h_ydlidar_x4){
	h_ydlidar_x4->cmd = CMD_STOP;
	h_ydlidar_x4->serial_drv.transmit((uint8_t *) &(h_ydlidar_x4->cmd), 2);
	return 0;
}

int ydlidar_x4_info(h_ydlidar_x4_t * h_ydlidar_x4){
	h_ydlidar_x4->cmd = CMD_INFO;
	h_ydlidar_x4->serial_drv.transmit((uint8_t *) &(h_ydlidar_x4->cmd), 2);
	return 0;
}

int ydlidar_x4_scan(h_ydlidar_x4_t * h_ydlidar_x4){
	h_ydlidar_x4->cmd = CMD_SCAN;
	h_ydlidar_x4->serial_drv.transmit((uint8_t *) &(h_ydlidar_x4->cmd), 2);
	return 0;
}

int ydlidar_x4_restart(h_ydlidar_x4_t * h_ydlidar_x4){
	h_ydlidar_x4->cmd = CMD_RESTART;
	h_ydlidar_x4->serial_drv.transmit((uint8_t *) &(h_ydlidar_x4->cmd), 2);
	return 0;
}

int ydlidar_x4_get_angle(h_ydlidar_x4_t * h_ydlidar_x4, uint16_t angle_LSB, uint16_t angle_MSB){
	uint16_t * angle;
	if(h_ydlidar_x4->decode_state == PARSING_START_ANGL){
		angle = &h_ydlidar_x4->start_angl;
	}
	else{
		angle = &h_ydlidar_x4->end_angl;
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

int ydlidar_x4_store_smpl(h_ydlidar_x4_t * h_ydlidar_x4){
	uint8_t smpl_idx=0;
	static uint16_t revoltion_idx=0;
	uint16_t angle_per_dist = (uint16_t) abs(h_ydlidar_x4->end_angl-h_ydlidar_x4->start_angl)/4;
	uint16_t first_angle=h_ydlidar_x4->start_angl;
	for(;smpl_idx<40;smpl_idx++,revoltion_idx++){
		if (revoltion_idx>= 600){
			revoltion_idx = 0;
		}
		if(h_ydlidar_x4->smpl[smpl_idx] > 0){
			h_ydlidar_x4->rev_smpls[revoltion_idx][0]=first_angle + (angle_per_dist*smpl_idx)/10;
			h_ydlidar_x4->rev_smpls[revoltion_idx][1]=h_ydlidar_x4->smpl[smpl_idx];
		}
	}
	ydlidar_x4_sort_smpl(h_ydlidar_x4, revoltion_idx);
	return 0;
}

int ydlidar_x4_irq_cb(h_ydlidar_x4_t * h_ydlidar_x4){
	ydlidar_x4_parsing_t * state = &h_ydlidar_x4->decode_state;
	uint8_t * dma_mem = h_ydlidar_x4->buf_DMA;
	uint8_t * frame_smpl = &h_ydlidar_x4->nb_smpl;
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
			case IDLE :
				if(last_byte == 0xA5 && dma_mem[idx_head] == 0x5A){
						*state = SCANNING;
						idx_head = 26;
				}
				else if(idx_head>26){
					*state = SCANNING;
				}
				break;

			case SCANNING :
				if(dma_mem[idx_head] == 0x55 && last_byte == 0xAA){
					*state = PARSING_SMPL;
					idx_limiter = 1;
				}
				break;

			case PARSING_SMPL :
				if(idx_limiter == 0){
					*frame_smpl = dma_mem[idx_head];
					*state 		= PARSING_START_ANGL;
					idx_limiter = 1;
				}
				else{
					idx_limiter --;
				}
				break;

			case PARSING_START_ANGL :
				if(idx_limiter == 0){
					ydlidar_x4_get_angle(h_ydlidar_x4, (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
					*state = PARSING_END_ANGL;
					idx_limiter = 1;
				}
				else{
					idx_limiter --;
				}
				break;

			case PARSING_END_ANGL :
				if(idx_limiter == 0){
					ydlidar_x4_get_angle(h_ydlidar_x4, (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
					*state = PARSING_DIST;
					idx_limiter = 0;
				}
				else{
					idx_limiter --;
				}
				break;
			case PARSING_DIST :
				if(((idx_limiter%2) != 0) && (idx_limiter < *frame_smpl)){
					ydlidar_x4_get_dist(&h_ydlidar_x4->smpl[idx_filler], (uint16_t) last_byte, (uint16_t) dma_mem[idx_head]);
					idx_filler++;
					idx_limiter ++;
				}
				else{
					idx_limiter ++;
					if(idx_limiter > *frame_smpl){
						idx_filler = 0;
						ydlidar_x4_store_smpl(h_ydlidar_x4);

						memset(h_ydlidar_x4->smpl,0,(*frame_smpl)*2);
						h_ydlidar_x4->start_angl=0;
						h_ydlidar_x4->end_angl=0;
						*frame_smpl = 0;
						*state = SCANNING;

					}
				}
				break;
		}
	}
	return 0;
}


int ydlidar_x4_sort_smpl(h_ydlidar_x4_t *h_ydlidar_x4, uint16_t revoltion_idx){

	uint16_t agl_idx, smpl_idx=0;
	uint16_t agl_inst[40] = {0};
	uint8_t nb_angle = 0;
	uint16_t dist;
	uint16_t agl;
	uint16_t min_dist;

	if(revoltion_idx < 40)
		return 1;

	for(smpl_idx = revoltion_idx-40;smpl_idx<revoltion_idx;smpl_idx++){

		if(h_ydlidar_x4->rev_smpls[smpl_idx][0]-(h_ydlidar_x4->rev_smpls[smpl_idx][0])%10 != agl_inst[nb_angle]){
			agl_inst[nb_angle+1] = h_ydlidar_x4->rev_smpls[smpl_idx][0]-(h_ydlidar_x4->rev_smpls[smpl_idx][0])%10;
			nb_angle++;

			 min_dist = 10000;

			for(agl_idx=revoltion_idx-40;agl_idx<revoltion_idx;agl_idx++){
				agl =  h_ydlidar_x4->rev_smpls[agl_idx][0];
				dist = h_ydlidar_x4->rev_smpls[agl_idx][1];
				if((agl - agl%10 == agl_inst[nb_angle]) && (dist < min_dist)){
					min_dist = dist;
				}
			}
			h_ydlidar_x4->sorted_dist[agl_inst[nb_angle]] = (min_dist + h_ydlidar_x4->sorted_dist[agl_inst[nb_angle]])/2;
		}

	}
	return 0;
}
