/*
 * imu.h
 *
 *  Created on: Nov 13, 2023
 *      Author: les_victor
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#define CALIB_GYRO_X 137
#define CALIB_GYRO_Y -275
#define CALIB_GYRO_Z -126



typedef int (* IMU_transmit_drv_t)(uint8_t *p_data, uint16_t size);
typedef int (* IMU_receive_drv_t)(uint8_t *p_data, uint16_t size);

typedef struct IMU_SPI_drv_struct
{
	IMU_transmit_drv_t transmit;
	IMU_receive_drv_t receive;
} IMU_SPI_drv_t;

typedef struct h_imu_drv_struct
{
	//Pointeur des fonctions pour utiliser le SPI
	IMU_SPI_drv_t spi_drv;

	// Données brutes
	int16_t accel[3];
	int16_t gyro[3];

	// Calibration de l'IMU (non utilisé)
	int16_t calib_gyro[3];

} h_imu_drv_t;


//extern imu_drv_t imu;




int imu_dev(h_imu_drv_t * h_imu);
int IMU_init(h_imu_drv_t * h_imu);
int IMU_gyro(h_imu_drv_t * h_imu);
int IMU_gyro2(h_imu_drv_t * h_imu);

int IMU_write8(h_imu_drv_t * h_imu, uint8_t reg, uint8_t *p_data);
int IMU_read8(h_imu_drv_t * h_imu, uint8_t reg, uint8_t *p_data);

int IMU_receive(uint8_t *p_data, uint16_t size);
int IMU_transmit(uint8_t *p_data, uint16_t size);

int16_t data_reconstruct(uint8_t lsb, uint8_t msb);

#endif /* INC_IMU_H_ */
