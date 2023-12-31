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
#define SPI_WRITE 0b00000000
#define SPI_READ 0b10000000

#define GYRO_X 0x22
#define GYRO_Y 0x24
#define GYRO_Z 0x26

#define GYRO_EN
#define ACCEL_EN

extern QueueHandle_t q_printf;

uint8_t receive_buffer = 0;
uint8_t transmit_buffer = 0x80;

//imu_drv_t imu;

int imu_dev(h_imu_drv_t * h_imu){
	printf("\r\n===== ADXL345 =====\r\n");
	printf("Connecter VCC sur le 5V et GND sur GND\r\n");
	printf("CS sur D3, SDO sur D12, SDA sur D11, SCL sur D13, INT1 sur D4\r\n");


	IMU_gyro(h_imu);

	printf("Lecture des accelerations :\r\n");
	printf("- Gyro X :%d\r\n", h_imu->gyro[0]);
	printf("- Gyro Y :%d\r\n", h_imu->gyro[1]);
	printf("- Gyro Z :%d\r\n", h_imu->gyro[2]);

	/*uint8_t msg [100];
	sprintf(msg, "Lecture des accelerations :\r\n- Gyro X :%d\r\n- Gyro Y :%d\r\n- Gyro Z :%d\r\n",
			imu.gyro[0], imu.gyro[1], imu.gyro[2]);
	xQueueSend(q_printf, (void *)msg, 5);*/

	return 0;
}

int IMU_init(h_imu_drv_t * h_imu){

	//Initialisation de la structure imu_drv_t
	h_imu->spi_drv.transmit = IMU_transmit;
	h_imu->spi_drv.receive = IMU_receive;
	h_imu->gyro[0] = 0;
	h_imu->gyro[1] = 0;
	h_imu->gyro[2] = 0;

	h_imu->calib_gyro[0]=CALIB_GYRO_X;
	h_imu->calib_gyro[1]=CALIB_GYRO_Y;
	h_imu->calib_gyro[2]= CALIB_GYRO_Z;

	receive_buffer = 0;
	transmit_buffer = 0xF;

	IMU_read8(h_imu, transmit_buffer, &receive_buffer);

	printf("WHO AM I ? = 0x%02X\r\n", receive_buffer);
	printf("Ou bien en decimal : %d\r\n", (int) receive_buffer);
	if (receive_buffer != 0b01101010)
	{
		printf("Le buffer ne vaut pas la valeur attendue\r\n");
		Error_Handler();
	}

#ifdef ACCEL_EN
	transmit_buffer = CTRL1_XL;
	IMU_read8(h_imu, transmit_buffer, &receive_buffer);
	receive_buffer = (receive_buffer & 0xF) | 0b1010<<4;
	IMU_write8(h_imu, transmit_buffer, &receive_buffer);

	printf("Accelerator successfully set to 6.6kHz\r\n");
#else
	//Lit le registre CTRL1_XL et écrit 0 sur les 4 bits de poids fort pour éteindre l'accéléromètre
	transmit_buffer = CTRL1_XL;
	IMU_read8(h_imu, transmit_buffer, &receive_buffer);
	receive_buffer = (receive_buffer & 0xF) | 0b0000<<4;
	IMU_write8(h_imu, transmit_buffer, &receive_buffer);

	printf("Accelerator successfully turned down\r\n");
#endif


#ifdef GYRO_EN
	//Lit le registre CTRL2_G et écrit sur les 4 bits de poids fort pour faire fonctionner le gyromètre à 12,5Hz
	transmit_buffer = CTRL2_G;
	IMU_read8(h_imu, transmit_buffer, &receive_buffer);
	//transmit_buffer += 0b10000000;
	receive_buffer = (receive_buffer & 0xF) | 0b1010<<4;
	IMU_write8(h_imu, transmit_buffer, &receive_buffer);

	printf("Gyroscope successfully set to 6.6kHz\r\n");
#else
	transmit_buffer = CTRL2_G;
	IMU_read8(h_imu, transmit_buffer, &receive_buffer);
	//transmit_buffer += 0b10000000;
	receive_buffer = (receive_buffer & 0xF) | 0b0000<<4;
	IMU_write8(h_imu, transmit_buffer, &receive_buffer);

	printf("Gyroscope successfully turned off\r\n");
#endif


	//Lecture des vitesses angulaires
	IMU_gyro(h_imu);
	printf("Lecture des accelerations :\r\n");
	printf("- Gyro X :%d\r\n", h_imu->gyro[0]);
	printf("- Gyro Y :%d\r\n", h_imu->gyro[1]);
	printf("- Gyro Z :%d\r\n", h_imu->gyro[2]);

	return 0;
}

int IMU_gyro(h_imu_drv_t * h_imu){
	uint8_t speed_low;
	uint8_t speed_high;
	//Vitesse en X
	IMU_read8(h_imu, GYRO_X, &speed_low);
	IMU_read8(h_imu, GYRO_X+1, &speed_high);
	h_imu->gyro[0] = h_imu->gyro[0]/2 + data_reconstruct(speed_low, speed_high)/2;// - h_imu->calib_gyro[0];
	//Vitesse en Y
	IMU_read8(h_imu, GYRO_Y, &speed_low);
	IMU_read8(h_imu, GYRO_Y+1, &speed_high);
	h_imu->gyro[1] = h_imu->gyro[1]/2 + data_reconstruct(speed_low, speed_high)/2;// - h_imu->calib_gyro[1];
	//Vitesse en Z
	IMU_read8(h_imu, GYRO_Z, &speed_low);
	IMU_read8(h_imu, GYRO_Z+1, &speed_high);
	h_imu->gyro[2] = h_imu->gyro[2]/2 + data_reconstruct(speed_low, speed_high)/2;// - h_imu->calib_gyro[2];
	return 0;
}


//Inutilisable tant que la sortie de l'IMU n'est pas carrée à tous les coups
int IMU_gyro2(h_imu_drv_t * h_imu){
	//uint8_t speed_low;
	//uint8_t speed_high;
	uint8_t speed[2] = {0,0};
	//Vitesse en X
	HAL_StatusTypeDef status;
	 uint8_t reg = GYRO_X | SPI_READ;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = h_imu->spi_drv.transmit(&reg, 1);
	status += h_imu->spi_drv.receive(speed, 2);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);
	h_imu->gyro[0] = (int16_t)((uint16_t)speed[0] + (((uint16_t)speed[1])<<8));// - h_imu->calib_gyro[0];
	//Vitesse en Y
	reg = GYRO_Y | SPI_READ;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = h_imu->spi_drv.transmit(&reg, 1);
	status += h_imu->spi_drv.receive(speed, 2);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);
	h_imu->gyro[1] = (int16_t)((uint16_t)speed[0] + (((uint16_t)speed[1])<<8));// - h_imu->calib_gyro[1];
	reg = GYRO_Z | SPI_READ;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = h_imu->spi_drv.transmit(&reg, 1);
	status += h_imu->spi_drv.receive(speed, 2);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);
	h_imu->gyro[2] = (int16_t)((uint16_t)speed[0] + (((uint16_t)speed[1])<<8));// - h_imu->calib_gyro[2];
	return 0;
}


//Fonction pour écrire sur 1 registre
int IMU_write8(h_imu_drv_t * h_imu, uint8_t reg, uint8_t *p_data){
	HAL_StatusTypeDef status;
	reg = reg | SPI_WRITE;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = h_imu->spi_drv.transmit(&reg, 1);
	status += h_imu->spi_drv.transmit(p_data, 1);
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_SET);
	return status;
}

//Fonction pour lire 1 registre
int IMU_read8(h_imu_drv_t * h_imu, uint8_t reg, uint8_t *p_data){
	HAL_StatusTypeDef status;
	reg = reg | SPI_READ;
	HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
	status = h_imu->spi_drv.transmit(&reg, 1);
	status += h_imu->spi_drv.receive(p_data, 1);
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

int16_t data_reconstruct(uint8_t lsb, uint8_t msb){
	return (int16_t)((uint16_t)lsb + (((uint16_t)msb)<<8));
}
