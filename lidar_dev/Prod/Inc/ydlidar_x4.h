#include "usart.h"
//1st header
#define START 0xA5
//2nd headers opcode
#define SCAN 0x60
#define STOP 0x65
#define INFO 0x90
#define STATUS 0x91
#define SOFT_RESTART 0x80


int Start_scan(UART_HandleTypeDef huart1);
