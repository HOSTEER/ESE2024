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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STACK_SIZE 1024
#define DEFAULT_TASK_PRIORITY 10
#define QUEUE_PRINTF_SIZE 100
#define QUEUE_PRINTF_LENGTH 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
QueueHandle_t q_printf = NULL;

TaskHandle_t h_task_init = NULL;
TaskHandle_t h_IMU_taskRead = NULL;
BaseType_t ret;

uint8_t odom_overflow = 0;
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

void task_init(void * unused)
{
	IMU_init();
	printf("Task init ok\r\n");
	for(;;){
		printf("Task init looping\r\n");
		imu_dev();
		HAL_GPIO_TogglePin(USER_LED3_GPIO_Port, USER_LED3_Pin);
		HAL_GPIO_TogglePin(USER_LED4_GPIO_Port, USER_LED4_Pin);
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
		__HAL_TIM_SET_COUNTER(&htim7, 0);
		HAL_TIM_Base_Start(&htim7);
		IMU_gyro(&imu);

		if(!odom_overflow){
			sprintf(msg, "Mesures de vitesse de rotation faite en %d us\r\n", time_lapped);
			xQueueSend(q_printf, (void *)msg, 5);
		}
		else{
			sprintf(msg, "Mesures de vitesse de rotation faite, mais la base de temps est corrompue (%d overflow(s))\r\n", odom_overflow);
			xQueueSend(q_printf, (void *)msg, 5);
		}

		//printf("Task init looping\r\n");
		HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
		vTaskDelay(5);
	}
}

void printfTask(void * unused)
{
	uint8_t msg[QUEUE_PRINTF_SIZE];
	BaseType_t ret;
	for(;;){
		ret = xQueueReceive(q_printf, (void *)msg, portMAX_DELAY);
		if(ret == pdTRUE){
			printf(msg);
		}
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
	ret = xTaskCreate(task_init, "task_init", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY, &h_task_init);
	if(ret != pdPASS)
	{
		printf("Could not create task init \r\n");
		Error_Handler();
	}

	ret = xTaskCreate(IMU_taskRead, "IMU_taskRead", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY + 1, &h_IMU_taskRead);
	if(ret != pdPASS)
	{
		printf("Could not create IMU taskRead \r\n");
		Error_Handler();
	}

	ret = xTaskCreate(printfTask, "printf_task", DEFAULT_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY - 1, &h_IMU_taskRead);
	if(ret != pdPASS)
	{
		printf("Could not create printf task\r\n");
		Error_Handler();
	}



	q_printf = xQueueCreate(QUEUE_PRINTF_LENGTH, QUEUE_PRINTF_SIZE);



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
