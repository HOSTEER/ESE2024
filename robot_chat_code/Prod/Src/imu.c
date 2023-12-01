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
#define SPI_WRITE 0b00000000;
#define SPI_READ 0b10000000;

uint8_t receive_buffer = 0;
uint8_t transmit_buffer = 0x80;

imu_drv_t imu;

int imu_dev(void){
	printf("\r\n===== ADXL345 =====\r\n");
	printf("Connecter VCC sur le 5V et GND sur GND\r\n");
	printf("CS sur D3, SDO sur D12, SDA sur D11, SCL sur D13, INT1 sur D4\r\n");





//	uint8_t buffer[6];
//	int16_t x_int;
//	int16_t y_int;
//	int16_t z_int;
//
//	float x, y, z;
	return 0;
}

int IMU_init(void){

	//Initialisation de la structure imu_drv_t
	imu.spi_drv.transmit = IMU_transmit;
	imu.spi_drv.receive = IMU_receive;
	imu.gyro[0] = 0;
	imu.gyro[1] = 0;
	imu.gyro[2] = 0;

	receive_buffer = 0;
	transmit_buffer = 0xF;

	IMU_read8(&imu, transmit_buffer, &receive_buffer);

	printf("WHO AM I ? = 0x%02X\r\n", receive_buffer);
	printf("Ou bien en decimal : %d\r\n", (int) receive_buffer);
	if (receive_buffer != 0b01101010)
	{
		printf("Le buffer ne vaut pas la valeur attendue\r\n");
		Error_Handler();
	}

	//Lit le registre CTRL1_XL et écrit 0 sur les 4 bits de poids fort pour éteindre l'accéléromètre
	transmit_buffer = CTRL1_XL;
	IMU_read8(&imu, transmit_buffer, &receive_buffer);

	receive_buffer = (receive_buffer & 0xF) + 0x00;

	IMU_write8(&imu, transmit_buffer, &receive_buffer);


	printf("Accelerator successfully turned down\r\n");

	//Lit le registre CTRL1_C et écrit sur les 4 bits de poids fort pour faire fonctionner le gyromètre à 12,5Hz
	transmit_buffer = CTRL2_G;
	IMU_read8(&imu, transmit_buffer, &receive_buffer);


	transmit_buffer += 0b10000000;
	IMU_write8(&imu, transmit_buffer, &receive_buffer);


	printf("Gyroscope successfully turned down\r\n");
	return 0;
}


//Fonction pour écrire sur 1 registre
int IMU_write8(imu_drv_t * imu, uint8_t reg, uint8_t *p_data){
	HAL_StatusTypeDef status;
	reg = reg | SPI_WRITE;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = imu->spi_drv.transmit(&reg, 1);
	status += imu->spi_drv.transmit(p_data, 1);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);
	return status;
}

//Fonction pour lire 1 registre
int IMU_read8(imu_drv_t * imu, uint8_t reg, uint8_t *p_data){
	HAL_StatusTypeDef status;
	reg = reg | SPI_READ;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = imu->spi_drv.transmit(&reg, 1);
	status += imu->spi_drv.receive(p_data, 1);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);
	return status;
}


int IMU_transmit(uint8_t *p_data, uint16_t size)
{

	return HAL_SPI_Transmit(&hspi1, p_data, size, HAL_MAX_DELAY);
}

int IMU_receive(uint8_t *p_data, uint16_t size)
{

	return HAL_SPI_Receive(&hspi1, p_data, 1, HAL_MAX_DELAY);
}
