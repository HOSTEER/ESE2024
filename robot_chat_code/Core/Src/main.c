/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ydlidar_x4.h"
#include "imu.h"
#include "motor.h"
#include "fixpoint_math.h"
#include "control.h"
#include "odometry.h"
#include "trajectoire.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	HUNTER			= 0x10,
	PREY			= 0x20,
	FALL_FORWARD 	= 0x00,
	FALL_BACKWARD 	= 0x01,
	COLL_FRONT 		= 0x02,
	COLL_BACK 		= 0x03,
	COLL_LEFT 		= 0x04,
	COLL_RIGHT 		= 0x05
}strat_mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_PRINTF_SIZE 100
#define QUEUE_PRINTF_LENGTH 10
#define DEFAULT_STACK_SIZE 512
#define DEFAULT_TASK_PRIORITY 1
#define DEFAULT_LIDAR_SPEED 950
#define TRUNC_FIXP 1000000000000

#define WHEEL_DIAMETER (43<<24) 	//43mm diameter, Q8.24
#define WHEEL_DIST (153<<16) 		//153mm distance between wheels, Q16.16
#define ENC_TICKSPERREV (400<<16)		//400 encoder ticks per revolution, Q16.16
#define ODOMETRY_FREQ 50 			//50Hz odometry refresh frequency
#define ROBOT_SPEED (500<<16) 		//robot speed, mm/s Q16.16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
QueueHandle_t q_printf = NULL;

TaskHandle_t h_task_init = NULL;
TaskHandle_t h_IMU_taskRead = NULL;
TaskHandle_t h_printf = NULL;
TaskHandle_t h_task_lidar = NULL;
TaskHandle_t h_task_lidar_ISR = NULL;
TaskHandle_t h_task_BT_and_Wire_RX_ISR = NULL;
TaskHandle_t h_task_BTN_ISR = NULL;
TaskHandle_t h_task_Motor = NULL;
TaskHandle_t h_task_MotorSpeed = NULL;

SemaphoreHandle_t lidar_RX_semaphore;
SemaphoreHandle_t Wire_BT_RX_semaphore;
SemaphoreHandle_t BTN_STATUS_semaphore;

BaseType_t ret;
h_ydlidar_x4_t lidar;
h_imu_drv_t h_imu;
uint8_t BT_RX;
uint8_t rx_pc;
uint8_t string_display[720];
hMotor_t Rmot;
hMotor_t Lmot;
hOdometry_t hOdometry;
int32_t mot_speed = 0;
int16_t cnt = 0;
int32_t angle = 0;
uint8_t odom_overflow = 0;
//vector_t test_vect = {-1*(2<<16),1<<16};
strat_mode_t strat_mode = PREY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	if(huart->Instance == USART1){
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(lidar_RX_semaphore, &xHigherPriorityTaskToken);
		portYIELD_FROM_ISR(xHigherPriorityTaskToken);
	}
	else if(huart->Instance == USART3 || huart->Instance == USART2){
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(Wire_BT_RX_semaphore, &xHigherPriorityTaskToken);
		portYIELD_FROM_ISR(xHigherPriorityTaskToken);
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(lidar_RX_semaphore, &xHigherPriorityTaskToken);
		portYIELD_FROM_ISR(xHigherPriorityTaskToken);
	}
}

int lidar_uart_transmit(uint8_t *p_data, uint16_t size)
{
	HAL_UART_Transmit(&huart1,p_data, size, HAL_MAX_DELAY);
	return 0;
}

int lidar_uart_receive(uint8_t *p_data)
{
	HAL_UART_Receive_DMA(&huart1,p_data, LIDAR2DMA_SIZE);
	return 0;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13) {
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(BTN_STATUS_semaphore, &xHigherPriorityTaskToken);
		portYIELD_FROM_ISR(xHigherPriorityTaskToken);
	}

	if(GPIO_Pin == FBD_EXTI1_Pin) {
		HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, 1);
	}
	if(GPIO_Pin == BBD_EXTI2_Pin) {
		HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, 1);
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == FBD_EXTI1_Pin) {
		HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, 0);
	}
	if(GPIO_Pin == BBD_EXTI2_Pin) {
		HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, 0);
	}
}


void task_init(void * unused)
{

	printf("Task init ok\r\n");
	for(;;){
		//imu_dev(&h_imu);
		//printf("Task init looping\r\n");
		HAL_GPIO_TogglePin(USER_LED0_GPIO_Port, USER_LED0_Pin);
		//HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
		//HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin);
		//HAL_GPIO_TogglePin(USER_LED3_GPIO_Port, USER_LED3_Pin);
		//HAL_GPIO_TogglePin(USER_LED4_GPIO_Port, USER_LED4_Pin);
		vTaskDelay(1000);
	}
}

void IMU_taskRead(void * unused)
{
	uint8_t msg [QUEUE_PRINTF_SIZE];
	uint32_t time_lapped;

	for(;;){
		//printf("Task init looping\r\n");
		time_lapped = __HAL_TIM_GET_COUNTER(&htim7);
		//__HAL_TIM_SET_COUNTER(&htim7, 0);
		//HAL_TIM_Base_Start(&htim7);
		IMU_gyro(&h_imu);

		/*uint8_t gyro_addr = 0x11;
		uint8_t value[3];
		IMU_read8(&h_imu, gyro_addr, value);*/

		/*if(!odom_overflow){
			sprintf(msg, "Mesures de vitesse de rotation faite en %d us\r\n", time_lapped);
			xQueueSend(q_printf, (void *)msg, 5);
		}
		else{
			sprintf(msg, "Mesures de vitesse de rotation faite, mais la base de temps est corrompue (%d overflow(s))\r\n", odom_overflow);
			xQueueSend(q_printf, (void *)msg, 5);
		}*/


		//sprintf(msg, "Lecture des accelerations :\r\n- Gyro X :%d\r\n- Gyro Y :%d\r\n- Gyro Z :%d\r\n", h_imu.gyro[0],  h_imu.gyro[1], h_imu.gyro[2]);
		//xQueueSend(q_printf, (void *)msg, 5);

		//printf("Task init looping\r\n");
		//HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
		vTaskDelay(5);
	}
}

void printfTask(void * unused)
{
	uint8_t msg[QUEUE_PRINTF_SIZE];
	BaseType_t ret;
	for(;;){
		ret = xQueueReceive(q_printf, (void *)msg, portMAX_DELAY);
		uint16_t msg_len = strlen(msg);
		if(ret == pdTRUE){
			//printf(msg);
			HAL_UART_Transmit_IT(&huart2, msg, msg_len);
		}
	}
}

void task_lidar(void * unused)
{
	printf("Task lidar ok\r\n");
	lidar.serial_drv.transmit = lidar_uart_transmit;
	lidar.serial_drv.receive = lidar_uart_receive;
	lidar.decode_state = SCANNING;
	lidar.serial_drv.receive(lidar.buf_DMA);
	lidar.nb_smpl = 0;
	lidar.start_angl = 0;
	lidar.end_angl = 0;
	HAL_GPIO_WritePin(LIDAR_RANGING_EN_GPIO_Port, LIDAR_RANGING_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LIDAR_EN_GPIO_Port, LIDAR_EN_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2, DEFAULT_LIDAR_SPEED-1);
	vTaskDelete(0);
	/*for(;;){
		if( xSemaphoreTake(lidar_RX_semaphore, 1000) == pdFALSE)
		{
			//HAL_NVIC_SystemReset();
		}
		ydlidar_x4_irq_cb(&lidar);
	}*/
}

void task_lidar_ISR(void * unused)
{
	printf("Task lidar ISR ok\r\n");
	for(;;){
		xSemaphoreTake(lidar_RX_semaphore, portMAX_DELAY );
		ydlidar_x4_irq_cb(&lidar);
		HAL_GPIO_TogglePin(USER_LED3_GPIO_Port, USER_LED3_Pin);
	}
}

void task_BT_and_Wire_RX_ISR(void * unused)
{
	printf("Task lidar ISR ok\r\n");
	HAL_UART_Receive_IT(&huart2, &rx_pc, 1);
	HAL_UART_Receive_IT(&huart3, &rx_pc, 1);
	for(;;){
		xSemaphoreTake(Wire_BT_RX_semaphore, portMAX_DELAY );
		if(rx_pc == 0xAA){
			rx_pc = 0xFF;
			for(int i=0;i<720;i++){
				if(i%2 == 0){
					string_display[i] = (lidar.sorted_dist[i>>1])>>8;
				}else{
					string_display[i] = (lidar.sorted_dist[i>>1]) & 0x00FF;
				}
			}
			HAL_UART_Transmit_DMA(&huart2,string_display, 720);
			HAL_UART_Transmit_DMA(&huart3,string_display, 720);
			HAL_UART_Receive_IT(&huart2, &rx_pc, 1);
			HAL_UART_Receive_IT(&huart3, &rx_pc, 1);
		}
		HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin);
	}
}

void task_BTN_ISR(void * unused)
{
	printf("Task BTN ok\r\n");
	for(;;){
		xSemaphoreTake(BTN_STATUS_semaphore, portMAX_DELAY);
		ydlidar_x4_scan(&lidar);
		HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
		//mot_speed ^= 1;
	}
}

void task_Motor(void * unused)
{
	vTaskDelay(10);
	motor_set_PWM(&Rmot, 0);
	motor_set_PWM(&Lmot, 0);
	Rmot.current_offset = Rmot.adc_dma_buff[Rmot.dma_buff_index];
	Lmot.current_offset = Lmot.adc_dma_buff[Lmot.dma_buff_index];
	int32_t speed = 0;
	int32_t angle = 0;
	uint32_t norm = 0;
	for(;;)
	{
		vTaskDelay(500);
		speed = Rmot.speed_measured[Rmot.speed_index];
		//CORDIC_vector(&test_vect);
		//angle = test_vect.angle*10;
		//norm = test_vect.norm*100;
		//printf("angle = %d, norme = %d\r\n", (int)angle/(1<<24), (int)norm/(1<<16));
		//printf("FWD %d\r\n",(int)(int16_t)__HAL_TIM_GET_COMPARE(Lmot.tim_FWD,TIM_CHANNEL_1));
		//printf("REV %d\r\n",(int)(int16_t)__HAL_TIM_GET_COMPARE(Lmot.tim_REV,TIM_CHANNEL_1));
		//printf("L Vdiff %d\r\n", (int)(fixed_div(Lmot.speed_output[Lmot.speed_index],128<<16,16) - fixed_div(Lmot.speed_measured[Lmot.speed_index],80<<16,16))/(1<<16));
		//printf("angle %d\r\n", (int)(hOdometry.angle*10/(1<<24)));
		//printf("dr %d\r\n", (int)(hOdometry.dr/(1<<24)));
		//printf("x %d, y %d\r\n", (int)hOdometry.x/(1<<16), (int)hOdometry.y/(1<<16));
		//printf("counts %d\r\n", (int)cnt);

		/*motor_set_PWM(&Rmot, 512);
		motor_set_PWM(&Lmot, 512);
		vTaskDelay(2000);
		motor_set_PWM(&Rmot, 0);
		motor_set_PWM(&Lmot, 0);
		vTaskDelay(2000);
		motor_set_PWM(&Rmot, -512);
		motor_set_PWM(&Lmot, -512);*/
	}
}

void task_MotorSpeed(void * unused)
{
	uint32_t V = 0;
	int32_t speed = 0;
	int32_t angle_corr = 0;
	int32_t Lspeed = 0;
	int32_t Rspeed = 0;
	for(;;)
	{
		//V = battery_get_voltage();
		cnt = (int16_t)__HAL_TIM_GET_COUNTER(Rmot.tim_ENC);
		odometry_update_pos(&hOdometry);
		//motor_get_speed(&Rmot);
		//motor_get_speed(&Lmot);
		//motor_get_current(&Rmot);
		//motor_get_current(&Lmot);
		if(hOdometry.x > 1000<<15)
		{
			angle += PI;
			hOdometry.x = 0;
		}
		if(hOdometry.y > 1000<<15)
		{
			angle += PI;
			hOdometry.y = 0;
			//mot_speed = 0;
		}
		if(hOdometry.x < -1000<<15)
		{
			angle += PI;
			hOdometry.x = 0;
		}
		if(hOdometry.y < -1000<<15)
		{
			angle += PI;
			hOdometry.y = 0;
			//mot_speed = 0;
		}
		if(angle > PI)
		{
			angle = -PI + angle%PI;
		}
		else if(angle < -PI)
		{
			angle = PI - angle%PI;
		}
		angle_corr = set_angle_corr(&hOdometry, angle);
		Rspeed = (int32_t)ROBOT_SPEED + fixed_mul((int32_t)ROBOT_SPEED, angle_corr, 24);
		Lspeed = (int32_t)ROBOT_SPEED - fixed_mul((int32_t)ROBOT_SPEED, angle_corr, 24);
		//speed = Rmot.speed_measured[Rmot.speed_index];
		//motor_set_PWM(&Rmot, 1024);
		//printf("vitesse moteur = %d.%u mm/s, courant moteur = %d.%u mA, tension batterie = %d.%u V\r\n", (int)(speed/(1<<16)), (unsigned int)conv_frac16_dec(speed & 0xFFFF,TRUNC_FIXP), (int)(Rmot.current_measured[Rmot.current_index]/(1<<16)), (unsigned int)conv_frac16_dec(Rmot.current_measured[Rmot.current_index] & 0xFFFF, TRUNC_FIXP) , (int)(V/(1<<16)),(unsigned int)conv_frac16_dec(V & 0xFFFF,TRUNC_FIXP));
		//printf("counts = %d, mesure : %d.%u mm/s, error : %d.%u mm/s, output : %d.%u, integral : %d.%u\r\n", (int)cnt, (int)(speed/(1<<16)), (unsigned int)conv_frac16_dec(speed & 0xFFFF, TRUNC_FIXP),(int)(Rmot.speed_error[Rmot.speed_index]/(1<<16)), (unsigned int)conv_frac16_dec((Rmot.speed_error[Rmot.speed_index]) & 0xFFFF, TRUNC_FIXP), (int)Rmot.speed_output[Rmot.speed_index]/(1<<16), (unsigned int)conv_frac16_dec(Rmot.speed_output[Rmot.speed_index] & 0xFFFF, TRUNC_FIXP), (int)(Rmot.speed_integral/(1<<16)), (unsigned int)conv_frac16_dec(Rmot.speed_integral & 0xFFFF, TRUNC_FIXP));
		set_speed_PID(&Rmot,mot_speed*Rspeed);	// 1<<14 = 150rpm , 1<<13 = 75rpm
		set_speed_PID(&Lmot,-(mot_speed*Lspeed));
		//printf("vitesse moteur = %d.%u rpm, courant moteur = %d.%u mA, tension batterie = %d.%u V\r\n", (int)(speed/(1<<16)), (unsigned int)conv_frac16_dec(speed & 0xFFFF,TRUNC_FIXP), (int)(Rmot.current_measured[Rmot.current_index]/(1<<16)), (unsigned int)conv_frac16_dec(Rmot.current_measured[Rmot.current_index] & 0xFFFF, TRUNC_FIXP) , (int)(V/(1<<16)),(unsigned int)conv_frac16_dec(V & 0xFFFF,TRUNC_FIXP));
		//printf("adc buffer 0: %d, 1: %d, 2: %d, i = %d, count = %d\r\n", (int)adcBuff[0],(int)adcBuff[1],(int)adcBuff[2], k, (int)__HAL_TIM_GET_COUNTER(&htim15));
		vTaskDelay(20);
	}
}

void fall_detection(void * unused){

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	lidar_RX_semaphore = xSemaphoreCreateBinary();
	Wire_BT_RX_semaphore = xSemaphoreCreateBinary();
	BTN_STATUS_semaphore = xSemaphoreCreateBinary();
	//q_printf = xQueueCreate(QUEUE_PRINTF_LENGTH, QUEUE_PRINTF_SIZE);

	HAL_TIM_PWM_Start_IT(&htim15,TIM_CHANNEL_1 | TIM_CHANNEL_2);

	current_sense_start();

	motor_init(&Rmot, &htim17, &htim14, &htim3, 1, 5<<16, 1<<14, 0, 1023<<16, 10, 0, 0, 0, 0);
	motor_init(&Lmot, &htim15, &htim16, &htim1, 2, 5<<16, 1<<14, 0, 1023<<16, 10, 0, 0, 0, 0);

	odometry_init(&hOdometry, &Rmot, &Lmot, WHEEL_DIAMETER, ENC_TICKSPERREV, WHEEL_DIST, ODOMETRY_FREQ);

	ret = xTaskCreate(task_init, "task_init", DEFAULT_STACK_SIZE/2, NULL, DEFAULT_TASK_PRIORITY, &h_task_init);
	if(ret != pdPASS)
	{
		printf("Could not create task init \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_lidar, "task_lidar", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY+2, &h_task_lidar);
	if(ret != pdPASS)
	{
		printf("Could not create task lidar \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_lidar_ISR, "task_lidar_ISR", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY+1, &h_task_lidar_ISR);
	if(ret != pdPASS)
	{
		printf("Could not create task lidar ISR \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_BT_and_Wire_RX_ISR, "task_BT_and_Wire_RX_ISR", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY, &h_task_BT_and_Wire_RX_ISR);
	if(ret != pdPASS)
	{
		printf("Could not create task BT ISR \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_BTN_ISR, "task_BTN_ISR", DEFAULT_STACK_SIZE/2, NULL, DEFAULT_TASK_PRIORITY, &h_task_BTN_ISR);
	if(ret != pdPASS)
	{
		printf("Could not create task BTN ISR \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_Motor, "task_Motor", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY+4, &h_task_Motor);
	if(ret != pdPASS)
	{
		printf("Could not create task Motor \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_MotorSpeed, "task_MotorSpeed", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY+3, &h_task_MotorSpeed);
	if(ret != pdPASS)
	{
		printf("Could not create task MotorSpeed \r\n");
		Error_Handler();
	}

	/*ret = xTaskCreate(IMU_taskRead, "IMU_taskRead", DEFAULT_STACK_SIZE+50, NULL, DEFAULT_TASK_PRIORITY + 9, &h_IMU_taskRead);
	if(ret != pdPASS)
	{
		printf("Could not create IMU taskRead \r\n");
		Error_Handler();
	}*/






	//IMU_init(&h_imu);

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM7) {
		odom_overflow++;
		HAL_GPIO_TogglePin(USER_LED0_GPIO_Port, USER_LED0_Pin);
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, 1);
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
