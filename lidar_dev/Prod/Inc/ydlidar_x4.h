//1st header
#define LIDAR_START 0xA5
//2nd headers opcode
#define LIDAR_SCAN 0x60
#define LIDAR_STOP 0x65
#define LIDAR_INFO 0x90
#define LIDAR_STATUS 0x91
#define LIDAR_SOFT_RESTART 0x80

extern uint8_t RXpacket;

int Lidar_start_scan(UART_HandleTypeDef * huart);
int Lidar_stop(UART_HandleTypeDef * huart);
int Lidar_info(UART_HandleTypeDef * huart);
int Lidar_scan(UART_HandleTypeDef * huart);
int Lidar_restart(UART_HandleTypeDef * huart);
