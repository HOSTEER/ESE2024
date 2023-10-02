#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "ydlidar_x4.h"

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

int ydlidar_x4_irq_cb(h_ylidar_x4_t * h_ylidar_x4){
	if(h_ylidar_x4->flag_scan == 0){
		if(h_ylidar_x4->buf_DMA[0] == 0xA5 && h_ylidar_x4->buf_DMA[1] == 0x5A){
				h_ylidar_x4->flag_scan = 1;
				h_ylidar_x4->idx_buf = 26;
		}
		else{
			return -1;
		}
	}
	if(h_ylidar_x4->flag_scan == 1){
		if(h_ylidar_x4->idx_buf < 90){
			while((h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] != 0xAA) && (h_ylidar_x4->idx_buf < 90) && (h_ylidar_x4->flag_AA == 0)){
				h_ylidar_x4->idx_buf++;
				if(h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] == 0xAA){
					h_ylidar_x4->flag_AA = 1;
				}
			}
			while((h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] != 0x55) && (h_ylidar_x4->idx_buf < 90) && (h_ylidar_x4->flag_55 == 0)){
				h_ylidar_x4->idx_buf++;
				if((h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] == 0x55) && (h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf - 1] == 0xAA)){
					h_ylidar_x4->flag_55 = 1;
				}
			}
			if(h_ylidar_x4->flag_55){
				if(((h_ylidar_x4->idx_buf + 2) < 90) && (h_ylidar_x4->nb_smpl == 0)){
					h_ylidar_x4->nb_smpl = h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf + 2];
					h_ylidar_x4->idx_buf +=2;
				}
				if(((h_ylidar_x4->idx_buf + 2) < 90) && (h_ylidar_x4->start_angl == 0)){
					h_ylidar_x4->start_angl = h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf + 1]>>1;
					uint16_t start_angle_MSB = (uint16_t) h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf + 2];
					h_ylidar_x4->start_angl = h_ylidar_x4->start_angl + (start_angle_MSB<<7);
					h_ylidar_x4->start_angl = h_ylidar_x4->start_angl>>6;
					h_ylidar_x4->idx_buf +=2;
				}
				if(((h_ylidar_x4->idx_buf + 2) < 90) && (h_ylidar_x4->end_angl == 0)){
					h_ylidar_x4->end_angl = h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf + 1]>>1;
					uint16_t end_angle_MSB = (uint16_t) h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf + 2];
					h_ylidar_x4->end_angl = h_ylidar_x4->end_angl + (end_angle_MSB<<7);
					h_ylidar_x4->end_angl = h_ylidar_x4->end_angl>>6;
					h_ylidar_x4->idx_buf +=2;
				}
			}
		}
		else{
			while((h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] != 0xAA) && (h_ylidar_x4->idx_buf < 90) && (h_ylidar_x4->flag_AA == 0)){
				h_ylidar_x4->idx_buf++;
				if(h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] == 0xAA){
					h_ylidar_x4->flag_AA = 1;
				}
			}
			while((h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] != 0x55) && (h_ylidar_x4->idx_buf < 90) && (h_ylidar_x4->flag_55 == 0)){
				h_ylidar_x4->idx_buf++;
				if((h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf] == 0x55) && (h_ylidar_x4->buf_DMA[h_ylidar_x4->idx_buf - 1] == 0xAA)){
					h_ylidar_x4->flag_55 = 1;
				}
			}
			while(h_ylidar_x4->flag_55){

			}
		}
	}
}
