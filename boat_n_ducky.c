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
 /* Definitions for emergency_timer */
osThreadId_t emergency_timerHandle;
const osThreadAttr_t emergency_timer_attributes = {
  .name = "emergency_timer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for boat */
osThreadId_t boatHandle;
const osThreadAttr_t boat_attributes = {
  .name = "boat",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for shooting_scene */
osThreadId_t shooting_sceneHandle;
const osThreadAttr_t shooting_scene_attributes = {
  .name = "shooting_scene",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void timer(void *argument);
void boat_running(void *argument);
void shoot(void *argument);

/* USER CODE BEGIN PFP */

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
  /* creation of emergency_timer */
  emergency_timerHandle = osThreadNew(timer, NULL, &emergency_timer_attributes);

  /* creation of boat */
  boatHandle = osThreadNew(boat_running, NULL, &boat_attributes);

  /* creation of shooting_scene */
  shooting_sceneHandle = osThreadNew(shoot, NULL, &shooting_scene_attributes);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, shoot_sound_Pin|froggy_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, sound_on_Pin|red_Pin|light_on_Pin|ducky_Pin
                          |boat_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : shoot_sound_Pin froggy_Pin */
  GPIO_InitStruct.Pin = shoot_sound_Pin|froggy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : shoot_point_Pin */
  GPIO_InitStruct.Pin = shoot_point_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(shoot_point_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : btn_Pin */
  GPIO_InitStruct.Pin = btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : sound_on_Pin red_Pin light_on_Pin ducky_Pin
                           boat_Pin */
  GPIO_InitStruct.Pin = sound_on_Pin|red_Pin|light_on_Pin|ducky_Pin
                          |boat_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : green_Pin */
  GPIO_InitStruct.Pin = green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(green_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_timer */
/**
  * @brief  Function implementing the emergency_timer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_timer */
void timer(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_boat_running */
/**
* @brief Function implementing the boat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_boat_running */
void boat_running(void *argument)
{
  /* USER CODE BEGIN boat_running */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    HAL_GPIO_WritePin(GPIOC, green_Pin, GPIO_PIN_SET);
    if(!HAL_GPIO_ReadPin(GPIOA, btn_Pin)){
    	HAL_GPIO_WritePin(GPIOC, green_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, red_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOB, sound_on_Pin, GPIO_PIN_SET);			// включаем звуковое сопровождение
    	HAL_GPIO_WritePin(GPIOB, light_on_Pin, GPIO_PIN_SET);			// и свет
    	int froggy_delay = 25000;


    		while(HAL_GPIO_ReadPin(GPIOC, shoot_point_Pin)){			// Ждем пока не замкнется геркон
    			osDelay(1);
    			HAL_GPIO_WritePin(GPIOB, boat_Pin, GPIO_PIN_SET);		// Лодка при этом едет

    			froggy_delay++;
    				if(froggy_delay >= 25000){
    					froggy_delay = 0;
    					HAL_GPIO_WritePin(GPIOC, froggy_Pin, GPIO_PIN_RESET);
    					osDelay(200);
    					HAL_GPIO_WritePin(GPIOC, froggy_Pin, GPIO_PIN_SET);
    				}
    		}

    		HAL_GPIO_WritePin(GPIOB, boat_Pin, GPIO_PIN_RESET);			// Останавливаем лодку
    		HAL_GPIO_WritePin(GPIOC, shoot_sound_Pin, GPIO_PIN_RESET);  // Прижимаем пин запуска выстрела к земле
    		osDelay(200);												// на 300 мсек
    		HAL_GPIO_WritePin(GPIOC, shoot_sound_Pin, GPIO_PIN_SET);	// и снова включаем

    		HAL_GPIO_WritePin(GPIOB, ducky_Pin, GPIO_PIN_SET);			// Летять утки
    		osDelay(8000);
    		HAL_GPIO_WritePin(GPIOB, boat_Pin, GPIO_PIN_SET);			// Через 8 сек поехала лодка
    		osDelay(2000);
    		HAL_GPIO_WritePin(GPIOB, ducky_Pin, GPIO_PIN_RESET);		//Через 2 сек утки успокоились
    		osDelay(25000);
    		HAL_GPIO_WritePin(GPIOB, boat_Pin, GPIO_PIN_RESET);			// Через 25 сек лодка останавилась
    		HAL_GPIO_WritePin(GPIOB, sound_on_Pin, GPIO_PIN_RESET);		// и звук тоже
    		HAL_GPIO_WritePin(GPIOB, light_on_Pin, GPIO_PIN_RESET);		// и свет

    	osDelay(120000);													// Задержка
    	//HAL_GPIO_WritePin(GPIOB, boat_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, red_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END boat_running */
}

/* USER CODE BEGIN Header_shoot */
/**
* @brief Function implementing the shooting_scene thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot */
void shoot(void *argument)
{
  /* USER CODE BEGIN shoot */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

  }
  /* USER CODE END shoot */
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
