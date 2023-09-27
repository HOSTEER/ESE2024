

#include <stdint.h>

typedef enum ylidar_x4_command_enum
{
	CMD_SCAN   	= 0xA560,
	CMD_STOP   	= 0xA565,
	CMD_INFO  	= 0xA590,
	CMD_RESTART	= 0xA591
} ylidar_x4_command_t;

typedef int (* ylidar_x4_transmit_drv_t)(uint8_t *p_data, uint16_t size);
typedef int (* ylidar_x4_receive_drv_t)(uint8_t *p_data, uint16_t size);

typedef struct ylidar_x4_serial_drv_struct
{
	ylidar_x4_transmit_drv_t transmit;
	ylidar_x4_receive_drv_t receive;
} ylidar_x4_serial_drv_t;

typedef struct h_ylidar_x4_struct
{
	// driver serial
	ylidar_x4_serial_drv_t serial_drv;
	// command available for transmit
	ylidar_x4_command_t cmd;
} h_ylidar_x4_t;

int ylidar_x4_stop(h_ylidar_x4_t * h_ylidar_x4);
int ylidar_x4_info(h_ylidar_x4_t * h_ylidar_x4);
int ylidar_x4_scan(h_ylidar_x4_t * h_ylidar_x4);
int ylidar_x4_restart(h_ylidar_x4_t * h_ylidar_x4);
