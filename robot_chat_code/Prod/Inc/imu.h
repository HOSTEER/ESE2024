/*
 * imu.h
 *
 *  Created on: Nov 13, 2023
 *      Author: les_victor
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

typedef int (* IMU_transmit_drv_t)(uint8_t *p_data, uint16_t size);
typedef int (* IMU_receive_drv_t)(uint8_t *p_data, uint16_t size);

typedef struct IMU_SPI_struct
{
	IMU_transmit_drv_t transmit;
	IMU_receive_drv_t receive;
} IMU_SPI_drv_t;

typedef struct IMU_drv_struct{
	//Pointeur des fonctions pour utiliser le SPI
	IMU_SPI_drv_t spi_drv;

	// Données brutes
	uint16_t accel[3];
	uint16_t gyro[3];

}imu_drv_t;

int imu_dev(void);
int IMU_init(void);

int IMU_write8(imu_drv_t * imu, uint8_t reg, uint8_t *p_data);
int IMU_read8(imu_drv_t * imu, uint8_t reg, uint8_t *p_data);

int IMU_receive(uint8_t *p_data, uint16_t size);
int IMU_transmit(uint8_t *p_data, uint16_t size);

#endif /* INC_IMU_H_ */
