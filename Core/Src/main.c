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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include<math.h>

#include <stdlib.h>

 #include <stdio.h>
#include "ds3231/DS3231.h"

#include "WS2812B/ws2812b.h"

#include "stdbool.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */



typedef struct
{
	uint8_t Sec;
	uint8_t Min;
	uint8_t Hour;
	uint8_t DaysOfWeek;
	uint8_t Date;
	uint8_t Month;
	uint8_t Year;
}QUEUE_RTC;

typedef struct
{
	uint8_t Sec;
	uint8_t Min;
	uint8_t Hour;
	uint8_t DaysOfWeek;
	uint8_t Date;
	uint8_t Month;
	uint8_t Year;
}QUEUE_RTC_NEW_TIME;

typedef struct
{
	uint8_t setValue;
	char name[4];
}QUEUE_SET_VALUE;


// RGB LEDs
extern uint8_t BYTESINPATTERN;
extern const bool digits_paterns[12][15];


//typedef struct{
//	uint8_t level_1 = 3;
//	uint8_t level_2 = 60;
//	uint8_t level_3 = 100;
//	uint8_t level_4 = 170;
//	uint8_t level_5 = 200;
//}level_brightness;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	ЗАВДАННЯ:



*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

/* Definitions for LightSensorTask */
osThreadId_t LightSensorTaskHandle;
const osThreadAttr_t LightSensorTask_attributes = {
  .name = "LightSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DC2321Task */
osThreadId_t DC2321TaskHandle;
uint32_t DC2321TaskBuffer[ 128 ];
osStaticThreadDef_t DC2321TaskControlBlock;
const osThreadAttr_t DC2321Task_attributes = {
  .name = "DC2321Task",
  .cb_mem = &DC2321TaskControlBlock,
  .cb_size = sizeof(DC2321TaskControlBlock),
  .stack_mem = &DC2321TaskBuffer[0],
  .stack_size = sizeof(DC2321TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ButtonsTask */
osThreadId_t ButtonsTaskHandle;
uint32_t EncoderTaskBuffer[ 128 ];
osStaticThreadDef_t EncoderTaskControlBlock;
const osThreadAttr_t ButtonsTask_attributes = {
  .name = "ButtonsTask",
  .cb_mem = &EncoderTaskControlBlock,
  .cb_size = sizeof(EncoderTaskControlBlock),
  .stack_mem = &EncoderTaskBuffer[0],
  .stack_size = sizeof(EncoderTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ws2812bTask */
osThreadId_t ws2812bTaskHandle;
uint32_t ws1228bTaskBuffer[ 400 ];
osStaticThreadDef_t ws1228bTaskControlBlock;
const osThreadAttr_t ws2812bTask_attributes = {
  .name = "ws2812bTask",
  .cb_mem = &ws1228bTaskControlBlock,
  .cb_size = sizeof(ws1228bTaskControlBlock),
  .stack_mem = &ws1228bTaskBuffer[0],
  .stack_size = sizeof(ws1228bTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LightTask */
osThreadId_t LightTaskHandle;
const osThreadAttr_t LightTask_attributes = {
  .name = "LightTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buttonPress */
osMessageQueueId_t buttonPressHandle;
const osMessageQueueAttr_t buttonPress_attributes = {
  .name = "buttonPress"
};
/* Definitions for readRTCQueue */
osMessageQueueId_t readRTCQueueHandle;
const osMessageQueueAttr_t readRTCQueue_attributes = {
  .name = "readRTCQueue"
};
/* Definitions for newRTCTimeQueue */
osMessageQueueId_t newRTCTimeQueueHandle;
const osMessageQueueAttr_t newRTCTimeQueue_attributes = {
  .name = "newRTCTimeQueue"
};
/* Definitions for setValueQueue */
osMessageQueueId_t setValueQueueHandle;
const osMessageQueueAttr_t setValueQueue_attributes = {
  .name = "setValueQueue"
};
/* Definitions for Light */
osMessageQueueId_t LightHandle;
const osMessageQueueAttr_t Light_attributes = {
  .name = "Light"
};
/* Definitions for RTCTrigSem */
osSemaphoreId_t RTCTrigSemHandle;
const osSemaphoreAttr_t RTCTrigSem_attributes = {
  .name = "RTCTrigSem"
};
/* USER CODE BEGIN PV */


// Create GLOBAL QUEUEs
QUEUE_RTC QUEUE_RTC_t;


int RandAlg_GetRandInt(int low, int high)
{
   return rand() % ((high + 1) - low) + low;
}

// -------------------------------------------------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartDC2321Task(void *argument);
void StartButtons(void *argument);
void Startws2812b(void *argument);
void StartLightTask(void *argument);

/* USER CODE BEGIN PFP */


// For button debounce
//uint32_t previousMillis = 0;
//uint32_t currentMillis = 0;

// ------------------------------------------------------------------------------
// Callback from keys (buttons 1,2,3 and 4 and trom RTC INT input).
// Callback starts timer for make debounce buttons.
// Callback from RTS interrupt occurs every one second.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static BaseType_t xHigherPriorityTaskWoken;

	if((GPIO_Pin == BUTTON_1_Pin) || (GPIO_Pin == BUTTON_2_Pin) || (GPIO_Pin == BUTTON_3_Pin) || (GPIO_Pin == BUTTON_4_Pin))
	{
		HAL_TIM_Base_Start_IT(&htim3);
	}
	if(GPIO_Pin == INT_RTC_Pin)
	{
		xSemaphoreGiveFromISR(RTCTrigSemHandle, &xHigherPriorityTaskWoken);
	}
}
// ------------------------------------------------------------------------------
uint8_t GetRand(uint8_t low, uint8_t high)
{
	return rand() % ((high + 1) - low) + low;
}
// ------------------------------------------------------------------------------
/*
@brief	Function generates random r, g and b, but sum all r, g and b equal sum.
 */
void generateRandNumbers(uint8_t *r, uint8_t *g, uint8_t *b, uint16_t sum)
{
	*r = (GetRand(0, sum))%(sum - 1) + 1;
	*g = (GetRand(0, sum))%(sum - *r);
	*b = (GetRand(0, sum))%(sum - *r - *g);
}
// ------------------------------------------------------------------------------
/*
@brief	Function chooses colors for time. Color depends on brightness and hour.
 */
void selectColors(uint8_t *r_result, uint8_t *g_result, uint8_t *b_result, uint16_t brightness, uint8_t hours)
{
	bool r = false;
	bool g = false;
	bool b = false;

	uint16_t total_channels = 3;


	uint16_t share = brightness/total_channels;
	uint16_t remainder = brightness % total_channels;

	switch(hours){
		case 6 ... 9:
			r = false, g = true, b = false;
			break;

		case 10 ... 13:
			r = false, g = true, b = true;
			break;

		case 14 ... 16:
			r = true, g = true, b = false;
			break;

		case 17 ... 19:
			r = true, g = false, b = true;
			break;

		case 20 ... 22:
			r = false, g = false, b = true;
			break;

		default:
			r = true, g = true, b = true;
			break;
	}

	*r_result = r ? share + remainder : 0;
	*g_result = g ? share + remainder : 0;
	*b_result = b ? share + remainder : 0;
}
// ------------------------------------------------------------------------------

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RTCTrigSem */
  RTCTrigSemHandle = osSemaphoreNew(1, 1, &RTCTrigSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of buttonPress */
  buttonPressHandle = osMessageQueueNew (3, sizeof(uint8_t), &buttonPress_attributes);

  /* creation of readRTCQueue */
  readRTCQueueHandle = osMessageQueueNew (1, sizeof(QUEUE_RTC), &readRTCQueue_attributes);

  /* creation of newRTCTimeQueue */
  newRTCTimeQueueHandle = osMessageQueueNew (1, sizeof(QUEUE_RTC_NEW_TIME), &newRTCTimeQueue_attributes);

  /* creation of setValueQueue */
  setValueQueueHandle = osMessageQueueNew (1, sizeof(QUEUE_SET_VALUE), &setValueQueue_attributes);

  /* creation of Light */
  LightHandle = osMessageQueueNew (1, sizeof(uint16_t), &Light_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LightSensorTask */
  LightSensorTaskHandle = osThreadNew(StartDefaultTask, NULL, &LightSensorTask_attributes);

  /* creation of DC2321Task */
  DC2321TaskHandle = osThreadNew(StartDC2321Task, NULL, &DC2321Task_attributes);

  /* creation of ButtonsTask */
  ButtonsTaskHandle = osThreadNew(StartButtons, NULL, &ButtonsTask_attributes);

  /* creation of ws2812bTask */
  ws2812bTaskHandle = osThreadNew(Startws2812b, NULL, &ws2812bTask_attributes);

  /* creation of LightTask */
  LightTaskHandle = osThreadNew(StartLightTask, NULL, &LightTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  HAL_TIM_Base_Start_IT(&htim3);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 150;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, USER_LED_Pin|USER_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_OUT_GPIO_Port, USER_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_LED_Pin USER_OUTPUT_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin|USER_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_1_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_4_Pin BUTTON_3_Pin BUTTON_2_Pin INT_RTC_Pin */
  GPIO_InitStruct.Pin = BUTTON_4_Pin|BUTTON_3_Pin|BUTTON_2_Pin|INT_RTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_OUT_Pin */
  GPIO_InitStruct.Pin = USER_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_OUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {
	  osDelay(1000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDC2321Task */
/**
* @brief Function implementing the DC2321Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDC2321Task */
void StartDC2321Task(void *argument)
{
  /* USER CODE BEGIN StartDC2321Task */
  /* Infinite loop */

	QUEUE_RTC_NEW_TIME QUEUE_RTC_NEW_TIME_t;
	uint8_t set_intwrrupt = 0;

	osDelay(4000);

	time_i2c_write_single( DS3231_I2C_ADDRESS, 14, &set_intwrrupt);							// Make 1 Hz generation interrupt from RTC modu4le

	osDelay(100);

	for(;;)
	{
		if(xSemaphoreTake(RTCTrigSemHandle, 0) == pdTRUE )									// If was interrupt from RTC read data from RTC
		{
			uint8_t buff= 0;
			uint8_t time[10] = {0};

			for(uint8_t i = 0; i <= 8; i++)
			{
				ds3231_read(i, &buff);														// Read data from RTC module
				time[i] = buff;
			}
			// Use only hours and minutes for show on screen
			QUEUE_RTC_t.Min = time[1];
			QUEUE_RTC_t.Hour = time[2];
			xQueueSend(readRTCQueueHandle, &QUEUE_RTC_t, 0);								// нSend read data to ws2812b task
		}

		if(xQueueReceive(newRTCTimeQueueHandle, &QUEUE_RTC_NEW_TIME_t, 0) == pdTRUE )		// If was selected new time sabe it into RTC module.
		{
			ds3231_set(0, &QUEUE_RTC_NEW_TIME_t.Sec);
			ds3231_set(1, &QUEUE_RTC_NEW_TIME_t.Min);
			ds3231_set(2, &QUEUE_RTC_NEW_TIME_t.Hour);
		}
  }
  /* USER CODE END StartDC2321Task */
}

/* USER CODE BEGIN Header_StartButtons */
/**
* @brief Function implementing the ButtonsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtons */
void StartButtons(void *argument)
{
  /* USER CODE BEGIN StartButtons */
  /* Infinite loop */
	osDelay(4000);

	QUEUE_SET_VALUE QUEUE_SET_VALUE_t;
	QUEUE_RTC_NEW_TIME QUEUE_RTC_NEW_TIME_t;

	uint8_t pressed_key = 0;
	char name[3] = {0};

	static int8_t setet_type = 1;

	static int8_t hour = 0;
	static int8_t minute = 0;
	static int8_t second = 0;

	static bool mode = false;

	enum{
		ENTER_BUTTON = 1,
		PLUS_BUTTON,
		MINUS_BUTTON,
		EXI_BUTTONT
	};

	for(;;)
	{
		if (xQueueReceive(buttonPressHandle , &pressed_key, 0 ) == pdTRUE)						// Read witch button was pressed (queue receiving from timer HAL_TIM_PeriodElapsedCallback)
		{
			if(pressed_key == ENTER_BUTTON)
			{
				mode = true;
			}

			if(mode == true )
			{
				osThreadSuspend(DC2321TaskHandle);												// Stop receive new time from RTC module

				switch(setet_type)
				{
					case 1:				// Set hours
						if(pressed_key == PLUS_BUTTON)
						{
							if(hour < 23)
							{
								hour++;
							}
							else
							{
								hour = 0;
							}

						}
						if(pressed_key == MINUS_BUTTON)
						{
							if(hour > 0)
							{
								hour--;
							}
							else
							{
								hour = 23;
							}
						}

						memset(QUEUE_SET_VALUE_t.name, 0, sizeof(QUEUE_SET_VALUE_t.name));
						sprintf(name, "%s","H:");

						QUEUE_SET_VALUE_t.setValue = hour;
						strcat(QUEUE_SET_VALUE_t.name, name);

						xQueueSend(setValueQueueHandle, &QUEUE_SET_VALUE_t, 0);

						if(pressed_key == EXI_BUTTONT)				// save data, and go to next settings
						{
							QUEUE_RTC_NEW_TIME_t.Hour = hour;
							setet_type++;
						}

					break;

					case 2:				// Set minutes
						if(pressed_key == PLUS_BUTTON)
							{
								if(minute < 59)
								{
									minute++;
								}
								else
								{
									minute = 0;
								}

							}
							if(pressed_key == MINUS_BUTTON)
							{
								if(minute > 0)
								{
									minute--;
								}
								else
								{
									minute = 59;
								}

							}

							memset(QUEUE_SET_VALUE_t.name, 0, sizeof(QUEUE_SET_VALUE_t.name));
							sprintf(name, "%s", "M:");

							QUEUE_SET_VALUE_t.setValue = minute;
							strcat(QUEUE_SET_VALUE_t.name, name);

							xQueueSend(setValueQueueHandle, &QUEUE_SET_VALUE_t, 0);

							if(pressed_key == EXI_BUTTONT)		// save data, and go to next settings
							{
								QUEUE_RTC_NEW_TIME_t.Min = minute;
								setet_type++;
							}
					break;

					case 3:				// Set seconds
						if(pressed_key == PLUS_BUTTON)
						{
							if(second < 59)
							{
								second++;
							}
							else
							{
								second = 0;
							}

						}
						if(pressed_key == MINUS_BUTTON)
						{
							if(second > 0)
							{
								second--;
							}
							else
							{
								second = 59;
							}
						}

						memset(QUEUE_SET_VALUE_t.name, 0, sizeof(QUEUE_SET_VALUE_t.name));
						sprintf(name, "%s", "S:");

						QUEUE_SET_VALUE_t.setValue = second;
						strcat(QUEUE_SET_VALUE_t.name, name);

						xQueueSend(setValueQueueHandle, &QUEUE_SET_VALUE_t, 0);

						if(pressed_key == EXI_BUTTONT)		// save data, and go to next settings
						{
							QUEUE_RTC_NEW_TIME_t.Sec = second;

							xQueueSend(newRTCTimeQueueHandle, &QUEUE_RTC_NEW_TIME_t, 0);
							osThreadResume(DC2321TaskHandle);

							mode = false;
							setet_type = 1;
						}
					break;
				}
			}

		}
		else
		{
			// If no data in the queue
			 osDelay(10);
		}
    osDelay(10);
  }
  /* USER CODE END StartButtons */
}

/* USER CODE BEGIN Header_Startws2812b */
/**
* @brief Function implementing the ws2812bTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startws2812b */
void Startws2812b(void *argument)
{
  /* USER CODE BEGIN Startws2812b */
  /* Infinite loop */

	QUEUE_RTC QUEUE_RTC_tt;
	QUEUE_SET_VALUE QUEUE_SET_VALUE_t;

	uint8_t first_time = 0;

	textAnimation();
	colorAnimation();

	for(;;)
	{
		uint16_t brightness = 0;

		uint8_t r_result = 0;
		uint8_t g_result = 0;
		uint8_t b_result = 0;

		if(xQueueReceive(readRTCQueueHandle, &QUEUE_RTC_tt, 0) == pdTRUE)								// Show current time
		{
			if(xQueueReceive(LightHandle, &brightness, 0)  == pdTRUE)									// Receive light data (from light sensor tssk)
			{
				if((QUEUE_RTC_tt.Hour >= 7) && (QUEUE_RTC_tt.Hour <= 22 ))								// if is the day hours, than brightness can't be less than 60
				{
					if(brightness <= 3 )							// If during day nature light is very low.
					{
						brightness = 60;							// Minimum brightness pear day
						selectColors(&r_result, &g_result, &b_result, brightness, QUEUE_RTC_tt.Hour);
					}
					else											// If light more than minimum light
					{
						selectColors(&r_result, &g_result, &b_result, brightness, QUEUE_RTC_tt.Hour);
					}
				}
				else																					// If is the night hours, then the brightness can be minimum
				{
					selectColors(&r_result, &g_result, &b_result, brightness, QUEUE_RTC_tt.Hour);
				}
				printTime(QUEUE_RTC_tt.Min, QUEUE_RTC_tt.Hour, r_result, g_result, b_result);
				first_time = 1;
			}
			else													// If no data in queue on the start
			{
				if(first_time == 0)
				{
					printTime(QUEUE_RTC_tt.Min, QUEUE_RTC_tt.Hour, 0, 200, 0);
				}
			}
		}

		if(xQueueReceive(setValueQueueHandle, &QUEUE_SET_VALUE_t, 0))            			// Show settings time
		{
			char buf[5] = {0};
			char num[3] = {0};
			char name[4] = {0};

			strcat(name, QUEUE_SET_VALUE_t.name);
			sprintf(num, "%d", QUEUE_SET_VALUE_t.setValue);

			strcat(buf, name);
			strcat(buf, num);

			cleanAllScreenBuffer();
			setStringOnScreenWithShift(0, buf, 4, 180, 180, 180);
			WS2812_Send();
		}
	}
  /* USER CODE END Startws2812b */
}

/* USER CODE BEGIN Header_StartLightTask */
/**
* @brief Function implementing the LightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightTask */
void StartLightTask(void *argument)
{
  /* USER CODE BEGIN StartLightTask */
  /* Infinite loop */

	osDelay(5000);

	uint8_t level_1 = 3;
	uint8_t level_2 = 60;
	uint8_t level_3 = 100;
	uint8_t level_4 = 130;
	uint8_t level_5 = 170;

	uint16_t brightnessLEDs = 0;
	int count = 0;
	int clobalLight = 0;

	for(;;)
	{
		brightnessLEDs = 0;

		taskENTER_CRITICAL();
		HAL_ADC_Start(&hadc1);
		uint32_t light = HAL_ADC_GetValue(&hadc1);
		taskEXIT_CRITICAL();

		if(count < 9)
		{
			count++;
			clobalLight = clobalLight + light;
		}
		else
		{
			light = clobalLight/10;

			clobalLight = 0;
			count = 0;


			if(light < 270)
			{
				brightnessLEDs = level_1;
			}
			if((light >= 270) && (light < 350))
			{
				brightnessLEDs = level_2;
			}
			if((light >= 350) && (light < 400))
			{
				brightnessLEDs = level_3;
			}
			if(light >= 400)
			{
				brightnessLEDs = level_5;
			}


//			if(light < 107)
//			{
//				brightnessLEDs = level_1;
//			}
//			if((light >= 107) && (light < 150))
//			{
//				brightnessLEDs = level_2;
//			}
//			if((light >= 150) && (light < 200))
//			{
//				brightnessLEDs = level_3;
//			}
//			if((light >= 200) && (light < 300))
//			{
//				brightnessLEDs = level_4;
//			}
//			if(light >= 300)
//			{
//				brightnessLEDs = level_5;
//			}

			xQueueSend(LightHandle, &brightnessLEDs, 0);
		}
      osDelay(100);
  }
  /* USER CODE END StartLightTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim == &htim3)
  {
	  BaseType_t xHigherPriorityTaskWoken;

  	uint8_t key_1 = 1;
  	uint8_t key_2 = 2;
  	uint8_t key_3 = 3;
  	uint8_t key_4 = 4;

  	if(HAL_GPIO_ReadPin(GPIOC, BUTTON_1_Pin) == GPIO_PIN_SET)
  	{
  		HAL_GPIO_TogglePin(GPIOC, USER_OUTPUT_Pin);
  		if((xQueueSendFromISR( buttonPressHandle, &key_1, &xHigherPriorityTaskWoken )) != 1)
  		{

  		}
  	}
  	else if(HAL_GPIO_ReadPin(GPIOA, BUTTON_2_Pin) == GPIO_PIN_SET)
  	{
  		HAL_GPIO_TogglePin(GPIOC, USER_OUTPUT_Pin);
  		if((xQueueSendFromISR( buttonPressHandle, &key_2, &xHigherPriorityTaskWoken )) != 1)
  		{

  		}
  	}
  	else if(HAL_GPIO_ReadPin(GPIOA, BUTTON_3_Pin) == GPIO_PIN_SET)
  	{
  		HAL_GPIO_TogglePin(GPIOC, USER_OUTPUT_Pin);
  		if((xQueueSendFromISR( buttonPressHandle, &key_3, &xHigherPriorityTaskWoken )) != 1)
  		{

  		}
  	}
  	else if(HAL_GPIO_ReadPin(GPIOA, BUTTON_4_Pin) == GPIO_PIN_SET)
  	{
  		HAL_GPIO_TogglePin(GPIOC, USER_OUTPUT_Pin);
  		if((xQueueSendFromISR( buttonPressHandle, &key_4, &xHigherPriorityTaskWoken )) != 1)
  		{

  		}
  	}
  	HAL_TIM_Base_Stop_IT(&htim3);
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
