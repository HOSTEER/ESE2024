#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "ydlidar_x4.h"

uint8_t RXpacket;
uint8_t TXpacket;

int Lidar_stop(UART_HandleTypeDef * huart){
	TXpacket = LIDAR_START;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	TXpacket = LIDAR_STOP;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	return 1;
}

int Lidar_info(UART_HandleTypeDef * huart){
	TXpacket = LIDAR_START;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	TXpacket = LIDAR_INFO;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	return 1;
}

int Lidar_scan(UART_HandleTypeDef * huart){
	TXpacket = LIDAR_START;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	TXpacket = LIDAR_SCAN;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	return 1;
}

int Lidar_restart(UART_HandleTypeDef * huart){
	TXpacket = LIDAR_START;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	TXpacket = LIDAR_SOFT_RESTART;
	HAL_UART_Transmit(huart,&TXpacket, 1, HAL_MAX_DELAY);
	return 1;
}
