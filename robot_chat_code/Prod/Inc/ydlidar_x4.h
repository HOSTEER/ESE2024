
#include "usart.h"
#include <stdint.h>

#define LIDAR2DMA_SIZE 194
#define LIDAR_DEFAULT_SMPL_NB 50

#define LITTLE_ENDIAN
//#define BIG_ENDIAN

#ifdef LITTLE_ENDIAN
typedef enum ydlidar_x4_command_enum
{
	CMD_SCAN   	= 0x60A5,
	CMD_STOP   	= 0x65A5,
	CMD_INFO  	= 0x90A5,
	CMD_RESTART	= 0x91A5
} ydlidar_x4_command_t;
#endif

#ifdef BIG_ENDIAN
typedef enum ydlidar_x4_command_enum
{
	CMD_SCAN   	= 0xA560,
	CMD_STOP   	= 0xA565,
	CMD_INFO  	= 0xA590,
	CMD_RESTART	= 0xA591
} ydlidar_x4_command_t;
#endif

typedef enum ydlidar_x4_parsing_enum
{
	SCANNING			= 1,
	PARSING_SMPL		= 2,
	PARSING_START_ANGL 	= 3,
	PARSING_END_ANGL	= 4,
	PARSING_DIST		= 5
} ydlidar_x4_parsing_t;

typedef int (* ydlidar_x4_transmit_drv_t)(uint8_t *p_data, uint16_t size);
typedef int (* ydlidar_x4_receive_drv_t)(uint8_t *p_data);

typedef struct ydlidar_x4_serial_drv_struct
{
	ydlidar_x4_transmit_drv_t transmit;
	ydlidar_x4_receive_drv_t receive;
} ydlidar_x4_serial_drv_t;

typedef struct h_ydlidar_x4_struct
{
	// driver serial
	ydlidar_x4_serial_drv_t serial_drv;
	// command available for transmit
	ydlidar_x4_command_t cmd;
	// 360 valeurs pour les 360 degre
	uint16_t smpl[LIDAR_DEFAULT_SMPL_NB];
	uint16_t sorted_dist[360];
	// Buffer pour stocker les valeur brut du DMA
	uint8_t buf_DMA[LIDAR2DMA_SIZE];
	ydlidar_x4_parsing_t decode_state;
	// bien demarre
	uint8_t nb_smpl;
	uint16_t start_angl;
	uint16_t end_angl;

} h_ydlidar_x4_t;

int ydlidar_x4_stop(h_ydlidar_x4_t * h_ydlidar_x4);
int ydlidar_x4_info(h_ydlidar_x4_t * h_ydlidar_x4);
int ydlidar_x4_scan(h_ydlidar_x4_t * h_ydlidar_x4);
int ydlidar_x4_restart(h_ydlidar_x4_t * h_ydlidar_x4);
int ydlidar_x4_irq_cb(h_ydlidar_x4_t * h_ydlidar_x4);
int ydlidar_x4_get_angle(h_ydlidar_x4_t * h_ydlidar_x4, uint16_t angle_LSB, uint16_t angle_MSB);
int ydlidar_x4_get_dist(uint16_t * dist, uint16_t dist_LSB, uint16_t dist_MSB);
int ydlidar_x4_sort_smpl(h_ydlidar_x4_t *h_ydlidar_x4, uint16_t revoltion_idx);
