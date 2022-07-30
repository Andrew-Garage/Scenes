/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 /* Definitions for white_car */
osThreadId_t white_carHandle;
const osThreadAttr_t white_car_attributes = {
  .name = "white_car",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blue_car */
osThreadId_t blue_carHandle;
const osThreadAttr_t blue_car_attributes = {
  .name = "blue_car",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for brat_go */
osThreadId_t brat_goHandle;
const osThreadAttr_t brat_go_attributes = {
  .name = "brat_go",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void wht_car(void *argument);
void bl_car(void *argument);
void brat(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t bro = 0;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of white_car */
  white_carHandle = osThreadNew(wht_car, NULL, &white_car_attributes);

  /* creation of blue_car */
  blue_carHandle = osThreadNew(bl_car, NULL, &blue_car_attributes);

  /* creation of brat_go */
  brat_goHandle = osThreadNew(brat, NULL, &brat_go_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, machinegun_light_Pin|shot_1_Pin|vubration_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, machinegun_Pin|headlamp_Pin|GPIO_PIN_10|GPIO_PIN_11
                          |shot_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(shot_2_GPIO_Port, shot_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : machinegun_light_Pin shot_1_Pin vubration_Pin */
  GPIO_InitStruct.Pin = machinegun_light_Pin|shot_1_Pin|vubration_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : machinegun_Pin headlamp_Pin PC10 PC11
                           shot_3_Pin */
  GPIO_InitStruct.Pin = machinegun_Pin|headlamp_Pin|GPIO_PIN_10|GPIO_PIN_11
                          |shot_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : shot_2_Pin */
  GPIO_InitStruct.Pin = shot_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(shot_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_wht_car */
/**
  * @brief  Function implementing the white_car thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_wht_car */
void wht_car(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if (bro == 1){
    HAL_GPIO_WritePin(headlamp_GPIO_Port, headlamp_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(machinegun_GPIO_Port, machinegun_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(machinegun_light_GPIO_Port, machinegun_light_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(vubration_GPIO_Port, vubration_Pin, GPIO_PIN_SET);
    osDelay(10);
    HAL_GPIO_WritePin(vubration_GPIO_Port, vubration_Pin, GPIO_PIN_RESET);
    osDelay(20);

    HAL_GPIO_WritePin(machinegun_GPIO_Port, machinegun_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(machinegun_light_GPIO_Port, machinegun_light_Pin, GPIO_PIN_RESET);
    osDelay(30);
    }

    HAL_GPIO_WritePin(machinegun_GPIO_Port, machinegun_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(headlamp_GPIO_Port, headlamp_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_bl_car */
/**
* @brief Function implementing the blue_car thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bl_car */
void bl_car(void *argument)
{
  /* USER CODE BEGIN bl_car */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(bro == 1){
    HAL_GPIO_WritePin(shot_1_GPIO_Port, shot_1_Pin, GPIO_PIN_SET);
    	osDelay(80);
    HAL_GPIO_WritePin(shot_1_GPIO_Port, shot_1_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(shot_2_GPIO_Port, shot_2_Pin, GPIO_PIN_SET);
        osDelay(80);
    HAL_GPIO_WritePin(shot_2_GPIO_Port, shot_2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(shot_3_GPIO_Port, shot_3_Pin, GPIO_PIN_SET);
        osDelay(80);
    HAL_GPIO_WritePin(shot_3_GPIO_Port, shot_3_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END bl_car */
}

/* USER CODE BEGIN Header_brat */
/**
* @brief Function implementing the brat_go thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_brat */
void brat(void *argument)
{
  /* USER CODE BEGIN brat */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); 			// ЗЕЛЕНЫЙ ВКЛ
    	  	 if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)){ 				//ЕСЛ�? КНОПКА НАЖАТА (стала притянута к земле)
    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);         // ЗЕЛЕНЫЙ ВЫКЛ
    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);               // КРАСНЫЙ ВКЛ

    				bro = 1;
    				osDelay(25000);
    				bro = 0;
    			osDelay(120000);						// ЖДЕМ
    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);	// КРАСНЫЙ ВЫКЛ
    		}
  }
  /* USER CODE END brat */
}

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
