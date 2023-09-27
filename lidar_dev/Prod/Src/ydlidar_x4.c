#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "ydlidar_x4.h"

int ylidar_x4_stop(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_SCAN;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) h_ylidar_x4->cmd, 2);
	return 0;
}

int ylidar_x4_info(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_INFO;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) h_ylidar_x4->cmd, 2);
	return 0;
}

int ylidar_x4_scan(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_SCAN;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) h_ylidar_x4->cmd, 2);
	return 0;
}

int ylidar_x4_restart(h_ylidar_x4_t * h_ylidar_x4){
	h_ylidar_x4->cmd = CMD_RESTART;
	h_ylidar_x4->serial_drv.transmit((uint8_t *) h_ylidar_x4->cmd, 2);
	return 0;
}
