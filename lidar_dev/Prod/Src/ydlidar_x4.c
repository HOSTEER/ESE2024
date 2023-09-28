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
	h_ylidar_x4->serial_drv.receive(h_ylidar_x4->first_frame , 19);
	h_ylidar_x4->serial_drv.transmit((uint8_t *) &(h_ylidar_x4->cmd), 2);
	//19 car on ne veut pas la premiere trame de 19 octets
	h_ylidar_x4->mode = SCAN_FRAMES;
	//on se prepare pour recupere les trames de donnees
	h_ylidar_x4->serial_drv.receive(h_ylidar_x4->rev_complete , 180);
	return 0;
}

int ylidar_x4_restart(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_RESTART;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) &(h_ylidar_x4->cmd), 2);
	return 0;
}

int ydlidar_x4_irq_cb(h_ylidar_x4_t * h_ylidar_x4){

}
