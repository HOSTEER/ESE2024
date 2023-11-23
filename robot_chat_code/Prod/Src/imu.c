/*
 * imu.c
 *
 *  Created on: Nov 13, 2023
 *      Author: les_victor
 */
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "imu.h"
#include "cmsis_os.h"
#include "spi.h"

#define CTRL1_XL 0x10
#define CTRL2_G  0x11

uint8_t receive_buffer = 0;
uint8_t transmit_buffer = 0x80;

int imu_dev(void){
	printf("\r\n===== ADXL345 =====\r\n");
	printf("Connecter VCC sur le 5V et GND sur GND\r\n");
	printf("CS sur D3, SDO sur D12, SDA sur D11, SCL sur D13, INT1 sur D4\r\n");

	receive_buffer = 0;
	transmit_buffer = 0xF;

	// Reading the WhoAmI register, it should be 0b01101010
	/*HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &receive_buffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);

	printf("WHO AM I ? = 0x%02X\r\n", receive_buffer);
	if (receive_buffer != 0b01101010)
	{
		printf("Le buffer ne vaut pas la valeur attendue\r\n");
		Error_Handler();
	}*/

	//Lit le registre CTRL1_XL et écrit 0 sur les 4 bits de poids fort pour éteindre l'accéléromètre
	transmit_buffer = CTRL1_XL;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &receive_buffer, 1, HAL_MAX_DELAY);

	transmit_buffer += 0b10000000;
	receive_buffer = receive_buffer & 0xF + 0x00;

	HAL_SPI_Transmit(&hspi1, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &receive_buffer, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);

	printf("Accelerator successfully turned down\r\n");

	//Lit le registre CTRL1_C et écrit sur les 4 bits de poids fort pour faire fonctionner le gyromètre à 12,5Hz
	transmit_buffer = CTRL2_G;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &receive_buffer, 1, HAL_MAX_DELAY);

	transmit_buffer += 0b10000000;
	receive_buffer = receive_buffer & 0xF + 0x01;

	HAL_SPI_Transmit(&hspi1, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &receive_buffer, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);

	printf("Gyroscope successfully turned down\r\n");

//	uint8_t buffer[6];
//	int16_t x_int;
//	int16_t y_int;
//	int16_t z_int;
//
//	float x, y, z;
	return 0;
}
