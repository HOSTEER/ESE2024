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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ydlidar_x4.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STACK_SIZE 512
#define DEFAULT_TASK_PRIORITY 1
#define DEFAULT_LIDAR_SPEED 85
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t h_task_lidar = NULL;
TaskHandle_t h_task_lidar_ISR = NULL;
TaskHandle_t h_task_BTN_ISR = NULL;
TaskHandle_t h_task_BT_and_Wire_RX_ISR = NULL;

SemaphoreHandle_t lidar_RX_semaphore;
SemaphoreHandle_t stm_RX_semaphore;
SemaphoreHandle_t BTN_STATUS_semaphore;


BaseType_t ret;

h_ydlidar_x4_t lidar;

uint8_t string_display[720];
uint8_t rx_pc = 0xFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	if(huart->Instance == USART1){
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(lidar_RX_semaphore, &xHigherPriorityTaskToken);
		portYIELD_FROM_ISR(xHigherPriorityTaskToken);
	}
	else if(huart->Instance == USART2){
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(stm_RX_semaphore, &xHigherPriorityTaskToken);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13) {
		BaseType_t xHigherPriorityTaskToken = pdFALSE;
		xSemaphoreGiveFromISR(BTN_STATUS_semaphore, &xHigherPriorityTaskToken);
		portYIELD_FROM_ISR(xHigherPriorityTaskToken);
	}
}

void task_lidar(void * unused)
{
	printf("Task lidar ok\r\n");
	lidar.serial_drv.transmit = lidar_uart_transmit;
	lidar.serial_drv.receive = lidar_uart_receive;
	lidar.decode_state = SCANNING;
	lidar.serial_drv.receive(lidar.buf_DMA);
	lidar.LSN = 0;
	lidar.start_angl = 0;
	lidar.end_angl = 0;
	//HAL_GPIO_WritePin(LIDAR_RANGING_EN_GPIO_Port, LIDAR_RANGING_EN_Pin, GPIO_PIN_SET); no need for lidar dev
	//HAL_GPIO_WritePin(LIDAR_EN_GPIO_Port, LIDAR_EN_Pin, GPIO_PIN_SET);
	//HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	//__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2, DEFAULT_LIDAR_SPEED-1);
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
	}
}

void task_BT_and_Wire_RX_ISR(void * unused)
{
	printf("Task lidar ISR ok\r\n");
	HAL_UART_Receive_IT(&huart2, &rx_pc, 1);
	for(;;){
		xSemaphoreTake(stm_RX_semaphore, portMAX_DELAY );
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
			HAL_UART_Receive_IT(&huart2, &rx_pc, 1);
		}
	}
}

void task_BTN_ISR(void * unused)
{
	printf("Task BTN ok\r\n");
	for(;;){
		xSemaphoreTake(BTN_STATUS_semaphore, portMAX_DELAY);
		ydlidar_x4_scan(&lidar);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	stm_RX_semaphore = xSemaphoreCreateBinary();
	lidar_RX_semaphore = xSemaphoreCreateBinary();
	BTN_STATUS_semaphore = xSemaphoreCreateBinary();
	ret = xTaskCreate(task_lidar, "task_lidar", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY+3, &h_task_lidar);
	if(ret != pdPASS)
	{
		printf("Could not create task lidar \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_lidar_ISR, "task_lidar_ISR", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY+2, &h_task_lidar_ISR);
	if(ret != pdPASS)
	{
		printf("Could not create task lidar ISR \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_BTN_ISR, "task_BTN_ISR", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY, &h_task_BTN_ISR);
	if(ret != pdPASS)
	{
		printf("Could not create task BTN ISR \r\n");
		Error_Handler();
	}
	ret = xTaskCreate(task_BT_and_Wire_RX_ISR, "task_BT_and_Wire_RX_ISR", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY, &h_task_BT_and_Wire_RX_ISR);
	if(ret != pdPASS)
	{
		printf("Could not create task BT ISR \r\n");
		Error_Handler();
	}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
