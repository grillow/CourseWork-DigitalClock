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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <led.h>
#include <stdbool.h>
#include "interface.h"
#include "index.html.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  bool rx_buffer_overflow;
  char rx_buffer[1];
} uart_state;

typedef enum {
  WIFI_STATE_INITIAL,
  WIFI_STATE_CWMODE_WAITING,
  WIFI_STATE_CIPMODE_WAITING,
  WIFI_STATE_CIPMUX_WAITING,
  WIFI_STATE_WAITING_WIFI_CREDENTIALS,
  WIFI_STATE_CWJAP_WAITING,
  WIFI_STATE_CIPSERVER_WAITING,
  WIFI_STATE_OPERATING,
} wifi_state_t;

typedef enum {
  TCP_SERVER_STATE_IDLE,
  TCP_SERVER_STATE_READING_CLIENT_ID,
  TCP_SERVER_STATE_READING_DATA_LENGTH,
  TCP_SERVER_STATE_READING_DATA,
  TCP_SERVER_STATE_SENDING_DATA,
} tcp_server_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static size_t process_uart_command(const char *command_buffer, char *command_response_buffer);
static void wifi_init(UART_HandleTypeDef *huart);
static void wifi_process_tcp_data(UART_HandleTypeDef *huart, uint32_t client, char *data, uint32_t data_length);
static size_t process_http_request(char *http_request_buffer, const size_t http_request_buffer_length, char *http_response_buffer);
static void GPIO_EXTI_LIGHT_SENSOR_D0();
static void SOUND_SENSOR_Callback(button_state_t old, button_state_t new);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uart_state command_uart = {false, {0}};
char command_buffer[256] = {0};
size_t command_buffer_index = 0;
char command_response_buffer[512];

uart_state wifi_uart = {false, {0}};
char wifi_response[1024] = {0};
size_t wifi_response_index = 0;
char wifi_tx_buffer[8192];
wifi_state_t wifi_state = WIFI_STATE_INITIAL;
tcp_server_state_t tcp_server_state = TCP_SERVER_STATE_IDLE;
char ssid[64] = {0};
char password[64] = {0};

uint32_t tcp_server_client_id = 0;
uint32_t tcp_server_to_receive = 0;
uint32_t tcp_server_received = 0;
char tcp_response_buffer[4096] = {0};
uint32_t tcp_response_length = 0;

// -1 - low, 0 - no change, 1 - high
int light_sensor_state = 0;
int light_sensor_state_requested = 0;

bool track_enabled = false;
uint32_t track_acc = 0;
uint32_t track_duration = 0;
button_t sound_sensor;

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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UART_Receive_IT(&huart2, (uint8_t*)command_uart.rx_buffer, sizeof(command_uart.rx_buffer)) != HAL_OK) {
    Error_Handler();
  }
  HAL_Delay(1000);
  if (HAL_GPIO_ReadPin(LIGHT_SENSOR_D0_GPIO_Port, LIGHT_SENSOR_D0_Pin) == GPIO_PIN_RESET) {
    light_sensor_state = -1;
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
      Error_Handler();
    }
  } else {
    light_sensor_state = 1;
  }
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init_interface();
  wifi_init(&huart1);

//  HAL_ADC_Start(&hadc1);
//  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//  const uint32_t value = HAL_ADC_GetValue(&hadc1);
  sound_sensor = button_create(
//      value <= SOUND_SENSOR_THRESHOLD ? SET : RESET,
      RESET,
      HAL_GetTick(),
      SOUND_SENSOR_BOUNCING_TIME_MS,
      &SOUND_SENSOR_Callback
  );
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    interface_update_buttons();
    HAL_ADC_Start_IT(&hadc1);

    HAL_Delay(5);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == '*') {
    // already initialized
    return;
  } else {
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, '*');
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_KEY_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_E_Pin|LED_D_Pin|LED_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_B_Pin|LED_KEY_3_Pin|LED_G_Pin|LED_F_Pin
                          |LED_A_Pin|LED_KEY_2_Pin|LED_KEY_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIGHT_SENSOR_D0_Pin */
  GPIO_InitStruct.Pin = LIGHT_SENSOR_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LIGHT_SENSOR_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_TOGGLE_STOPWATCH_Pin BUTTON_TOGGLE_DISPLAY_Pin BUTTON_INCREASE_Pin BUTTON_DECREASE_Pin */
  GPIO_InitStruct.Pin = BUTTON_TOGGLE_STOPWATCH_Pin|BUTTON_TOGGLE_DISPLAY_Pin|BUTTON_INCREASE_Pin|BUTTON_DECREASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED_KEY_4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_KEY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_TOGGLE_SET_TIME_Pin */
  GPIO_InitStruct.Pin = BUTTON_TOGGLE_SET_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_TOGGLE_SET_TIME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_E_Pin LED_D_Pin LED_C_Pin */
  GPIO_InitStruct.Pin = LED_E_Pin|LED_D_Pin|LED_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin LED_KEY_3_Pin LED_G_Pin LED_F_Pin
                           LED_A_Pin LED_KEY_2_Pin LED_KEY_1_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_KEY_3_Pin|LED_G_Pin|LED_F_Pin
                          |LED_A_Pin|LED_KEY_2_Pin|LED_KEY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    const char received_byte = wifi_uart.rx_buffer[0];

    wifi_response[wifi_response_index++] = received_byte;
    if (wifi_response_index >= sizeof(wifi_response)) {
      if (!wifi_uart.rx_buffer_overflow) {
        wifi_uart.rx_buffer_overflow = true;
        wifi_response_index = 0;
        memset(wifi_response, 0, sizeof(wifi_response));
      }
    }

    switch (wifi_state) {
    case WIFI_STATE_INITIAL: break;
    case WIFI_STATE_CWMODE_WAITING:
      if(wifi_response_index >= 4 && !strcmp(wifi_response + wifi_response_index - 4, "OK\r\n")) {
        wifi_state = WIFI_STATE_CIPMODE_WAITING;
        const int len = sprintf(wifi_tx_buffer, "AT+CIPMODE=0\r\n");
        HAL_UART_Transmit_IT(huart, (uint8_t*)wifi_tx_buffer, len);
        wifi_response_index = 0;
        memset(wifi_response, 0, sizeof(wifi_response));
      }
      break;
    case WIFI_STATE_CIPMODE_WAITING:
      if(wifi_response_index >= 4 && !strcmp(wifi_response + wifi_response_index - 4, "OK\r\n")) {
        wifi_state = WIFI_STATE_CIPMUX_WAITING;
        const int len = sprintf(wifi_tx_buffer, "AT+CIPMUX=1\r\n");
        HAL_UART_Transmit_IT(huart, (uint8_t*)wifi_tx_buffer, len);
        wifi_response_index = 0;
        memset(wifi_response, 0, sizeof(wifi_response));
      }
      break;
    case WIFI_STATE_CIPMUX_WAITING:
      if(wifi_response_index >= 4 && !strcmp(wifi_response + wifi_response_index - 4, "OK\r\n")) {
        wifi_state = WIFI_STATE_WAITING_WIFI_CREDENTIALS;
        wifi_response_index = 0;
        memset(wifi_response, 0, sizeof(wifi_response));

        const int len = sprintf(command_response_buffer, "Ready to connect to Wi-Fi\n");
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)command_response_buffer, len);
      }
      break;
    case WIFI_STATE_WAITING_WIFI_CREDENTIALS: break;
    case WIFI_STATE_CWJAP_WAITING:
      if(wifi_response_index >= 4 && !strcmp(wifi_response + wifi_response_index - 4, "OK\r\n")) {
        wifi_state = WIFI_STATE_CIPSERVER_WAITING;
        const int len = sprintf(wifi_tx_buffer, "AT+CIPSERVER=1,80\r\n");
        HAL_UART_Transmit_IT(huart, (uint8_t*)wifi_tx_buffer, len);
        wifi_response_index = 0;
        memset(wifi_response, 0, sizeof(wifi_response));
      }
      break;
    case WIFI_STATE_CIPSERVER_WAITING:
      if(wifi_response_index >= 4 && !strcmp(wifi_response + wifi_response_index - 4, "OK\r\n")) {
        wifi_state = WIFI_STATE_OPERATING;
        wifi_response_index = 0;
        memset(wifi_response, 0, sizeof(wifi_response));

        const int len = sprintf(command_response_buffer, "Web-server ready\n");
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)command_response_buffer, len);
      }
      break;
    case WIFI_STATE_OPERATING:
      switch (tcp_server_state) {
      case TCP_SERVER_STATE_IDLE:
        if(wifi_response_index >= 10 && !strcmp(wifi_response + wifi_response_index - 10, ",CONNECT\r\n")) {
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        } else if (wifi_response_index >= 9 && !strcmp(wifi_response + wifi_response_index - 9, ",CLOSED\r\n")) {
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        } else if (wifi_response_index >= 9 && !strcmp(wifi_response + wifi_response_index - 9, "SEND OK\r\n")) {
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        } else if (wifi_response_index >= 5 && !strcmp(wifi_response + wifi_response_index - 5, "+IPD,")) {
          tcp_server_state = TCP_SERVER_STATE_READING_CLIENT_ID;
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        }
        break;
      case TCP_SERVER_STATE_READING_CLIENT_ID:
        if (received_byte == ',') {
          tcp_server_state = TCP_SERVER_STATE_READING_DATA_LENGTH;
          wifi_response[wifi_response_index - 1] = '\0';
          sscanf(wifi_response, "%"SCNu32"", &tcp_server_client_id);
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        }
        break;
      case TCP_SERVER_STATE_READING_DATA_LENGTH:
        if (received_byte == ':') {
          tcp_server_state = TCP_SERVER_STATE_READING_DATA;
          wifi_response[wifi_response_index - 1] = '\0';
          sscanf(wifi_response, "%"SCNu32"", &tcp_server_to_receive);
          tcp_server_received = 0;
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        }
        break;
      case TCP_SERVER_STATE_READING_DATA:
        if (++tcp_server_received == tcp_server_to_receive) {
          tcp_server_state = TCP_SERVER_STATE_IDLE;
          wifi_process_tcp_data(huart, tcp_server_client_id, wifi_response, tcp_server_received);
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        }
        break;
      case TCP_SERVER_STATE_SENDING_DATA:
        if(wifi_response_index >= 6 && !strcmp(wifi_response + wifi_response_index - 6, "OK\r\n> ")) {
          tcp_server_state = TCP_SERVER_STATE_IDLE;
          HAL_UART_Transmit_IT(huart, (uint8_t*)tcp_response_buffer, tcp_response_length);
          wifi_response_index = 0;
          memset(wifi_response, 0, sizeof(wifi_response));
        }
        break;
      }
      break;
    }

    HAL_UART_Receive_IT(huart, (uint8_t*)wifi_uart.rx_buffer, sizeof(wifi_uart.rx_buffer));
  } else if (huart->Instance == USART2) {
    const char received_byte = command_uart.rx_buffer[0];
    switch (received_byte) {
    default:
      if (command_buffer_index >= sizeof(command_buffer)) {
        if (!command_uart.rx_buffer_overflow) {
          command_uart.rx_buffer_overflow = true;
          const int len = sprintf(command_response_buffer, "Error: command shouldn't be longer than %d\n", sizeof(command_buffer));
          HAL_UART_Transmit_IT(huart, (uint8_t*)command_response_buffer, len);
          command_buffer_index = 0;
          memset(command_buffer, 0, sizeof(command_buffer));
        }
      } else {
        command_buffer[command_buffer_index++] = received_byte;
      }
      break;
    case '\n':
      command_buffer[command_buffer_index] = '\0';
      command_buffer_index = 0;
      if (!command_uart.rx_buffer_overflow) {
        const size_t command_response_length = process_uart_command(command_buffer, command_response_buffer);
        HAL_UART_Transmit_IT(huart, (uint8_t*)command_response_buffer, command_response_length);
      }
      command_uart.rx_buffer_overflow = false;
      memset(command_buffer, 0, sizeof(command_buffer));
    }

    HAL_UART_Receive_IT(huart, (uint8_t*)command_uart.rx_buffer, sizeof(command_uart.rx_buffer));
  } else if (huart->Instance == USART3) {

  }
}

size_t process_uart_command(const char *command_buffer, char *command_response_buffer)
{
  char command[sizeof(command_buffer)] = {0};

  if (sscanf(command_buffer, "%s ", command) != 1) {
    return sprintf(command_response_buffer, "Error: cannot parse: %s\n", command_buffer);
  }

  if (!strcmp(command, "help")) {
    return sprintf(command_response_buffer, "Commands:\nhelp\nget_time\nset_time <hh> <mm> <ss>\nget_daylight_time\nget_stopwatch\ntrack_enable\ntrack_disable\nget_track\nconnect_wifi \"<ssid>\" \"<password>\"\n");
  } else if (!strcmp(command, "get_time")) {
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;   // it won't work without reading date
    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      return sprintf(command_response_buffer, "Error: could not get time\n");
    }
    return sprintf(command_response_buffer, "Time: %02d:%02d:%02d\n", time.Hours, time.Minutes, time.Seconds);
  } else if (!strcmp(command, "set_time")) {
    int hours = 0;
    int minutes = 0;
    int seconds = 0;
    const size_t offset = 9;
    if (sscanf(command_buffer + offset, "%d %d %d", &hours, &minutes, &seconds) != 3) {
      return sprintf(command_response_buffer, "Error: cannot parse hh mm ss: %s\n", command_buffer + offset);
    }
    if (hours < 0 || hours >= 24) {
      return sprintf(command_response_buffer, "Error: wrong hours value: %d\n", hours);
    }
    if (minutes < 0 || minutes >= 60) {
      return sprintf(command_response_buffer, "Error: wrong minutes value: %d\n", minutes);
    }
    if (seconds < 0 || seconds >= 60) {
      return sprintf(command_response_buffer, "Error: wrong seconds value: %d\n", seconds);
    }
    // could sscanf directly to this struct, but it would break data validation
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;   // it won't work without reading date
    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      return sprintf(command_response_buffer, "Error: could not set time\n");
    }
    time.Hours = hours;
    time.Minutes = minutes;
    time.Seconds = seconds;
    if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      return sprintf(command_response_buffer, "Error: could not set time\n");
    }
    return sprintf(command_response_buffer, "Time set to %02d:%02d:%02d\n", hours, minutes, seconds);
  } else if (!strcmp(command, "get_daylight_time")) {
    uint32_t light_duration = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_DATA_ADDRESS, 2, (uint8_t*)&light_duration, sizeof(light_duration), HAL_MAX_DELAY) != HAL_OK) {
      //oy vey (HAL_BUSY)
    }
    const uint32_t seconds = light_duration / 10000;
    const uint32_t minutes = seconds / 60;
    const uint32_t hours   = minutes / 60;
    return sprintf(command_response_buffer, "Daylight time: %02"PRIu32":%02"PRIu32":%02"PRIu32"\n", hours, minutes % 60, seconds % 60);
  } else if (!strcmp(command, "get_stopwatch")) {
    const uint32_t total_centiseconds = interface.stopwatch_mode.stopwatch / 100;
    const uint32_t total_seconds      = total_centiseconds / 100;
    const uint32_t total_minutes      = total_seconds / 60;
    const uint32_t total_hours        = total_minutes / 60;

    const uint32_t centiseconds       = total_centiseconds % 100;
    const uint32_t seconds            = total_seconds % 60;
    const uint32_t minutes            = total_minutes % 60;
    const uint32_t hours              = total_hours;
    return sprintf(command_response_buffer, "Last stopwatch: %02"PRIu32":%02"PRIu32":%02"PRIu32".%02"PRIu32"\n", hours, minutes, seconds, centiseconds);
  } else if (!strcmp(command, "track_enable")) {
    track_enabled = true;
    return sprintf(command_response_buffer, "Track sampling enabled\n");
  } else if (!strcmp(command, "track_disable")) {
    track_enabled = false;
    return sprintf(command_response_buffer, "Track sampling disabled\n");
  } else if (!strcmp(command, "get_track")) {
    const uint32_t total_centiseconds = track_duration / 100;
    const uint32_t total_seconds      = total_centiseconds / 100;
    const uint32_t total_minutes      = total_seconds / 60;
    const uint32_t total_hours        = total_minutes / 60;

    const uint32_t centiseconds       = total_centiseconds % 100;
    const uint32_t seconds            = total_seconds % 60;
    const uint32_t minutes            = total_minutes % 60;
    const uint32_t hours              = total_hours;
    return sprintf(command_response_buffer, "Track duration: %02"PRIu32":%02"PRIu32":%02"PRIu32".%02"PRIu32"\n", hours, minutes, seconds, centiseconds);
  } else if (!strcmp(command, "connect_wifi")) {
    if (wifi_state != WIFI_STATE_WAITING_WIFI_CREDENTIALS) {
      return sprintf(command_response_buffer, "Error: Wi-Fi module not ready\n");
    }
    const size_t offset = 13;
    if (sscanf(command_buffer + offset, "\"%[^\"]\" \"%[^\"]\"", ssid, password) != 2) {
      return sprintf(command_response_buffer, "Error: cannot parse \"<ssid>\" \"<password>\": %s\n", command_buffer + offset);
    }
    wifi_state = WIFI_STATE_CWJAP_WAITING;
    const int len = sprintf(wifi_tx_buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)wifi_tx_buffer, len);
    return sprintf(command_response_buffer, "Connecting to Wi-Fi\n");
  } else {
    return sprintf(command_response_buffer, "Error: unknown command: %s\n", command);
  }
}

void wifi_init(UART_HandleTypeDef *huart)
{
  int len = sprintf(wifi_tx_buffer, "AT+CWQAP\r\n");
  HAL_UART_Transmit(huart, (uint8_t*)wifi_tx_buffer, len, HAL_MAX_DELAY);
  HAL_Delay(1000);
  len = sprintf(wifi_tx_buffer, "AT+RST\r\n");
  HAL_UART_Transmit(huart, (uint8_t*)wifi_tx_buffer, len, HAL_MAX_DELAY);
  HAL_Delay(1000);
  len = sprintf(wifi_tx_buffer, "AT+RESTORE\r\n");
  HAL_UART_Transmit(huart, (uint8_t*)wifi_tx_buffer, len, HAL_MAX_DELAY);
  HAL_Delay(1000);
  len = sprintf(wifi_tx_buffer, "ATE0\r\n");
  HAL_UART_Transmit(huart, (uint8_t*)wifi_tx_buffer, len, HAL_MAX_DELAY);

  HAL_Delay(3000);

  wifi_uart.rx_buffer_overflow = false;
  wifi_response_index = 0;
  memset(wifi_response, 0, sizeof(wifi_response));

  wifi_state = WIFI_STATE_CWMODE_WAITING;
  len = sprintf(wifi_tx_buffer, "AT+CWMODE=1\r\n");
  HAL_UART_Transmit_IT(huart, (uint8_t*)wifi_tx_buffer, len);
  HAL_UART_Receive_IT(huart, (uint8_t*)wifi_uart.rx_buffer, sizeof(wifi_uart.rx_buffer));
}

void wifi_process_tcp_data(UART_HandleTypeDef *huart, uint32_t client, char *data, uint32_t data_length)
{
  tcp_response_length = process_http_request(wifi_response, data_length, tcp_response_buffer);

  tcp_server_state = TCP_SERVER_STATE_SENDING_DATA;

  const int len = sprintf(wifi_tx_buffer, "AT+CIPSEND=%"PRIu32",%"PRIu32"\r\n", client, tcp_response_length);
  HAL_UART_Transmit_IT(huart, (uint8_t*)wifi_tx_buffer, len);
}

size_t process_http_request(char *http_request_buffer, const size_t http_request_buffer_length, char *http_response_buffer)
{
  const char *type = http_request_buffer;
  char *type_end = strchr(type, ' ');
  *type_end = '\0';

  const char *path = type_end + 1;
  char *path_end = strchr(path, ' ');
  *path_end = '\0';

  const char *data = strstr(path_end + 1, "\r\n\r\n") + 4;
  char *data_end = http_request_buffer + http_request_buffer_length;
  *data_end = '\0';

  if (strcmp(type, "GET") == 0) {
      if (strcmp(path, "/") == 0) {
        const size_t http_response_length_pre = sprintf(http_response_buffer, "HTTP/1.1 200 OK\r\nContent-Encoding: gzip\r\nContent-Type: text/html; charset=UTF-8\r\nContent-Length: %"PRIu32"\r\n\r\n", (uint32_t)index_html_length);
        memcpy(http_response_buffer + http_response_length_pre, index_html, index_html_length);
        return http_response_length_pre + index_html_length;
      }
      if (strcmp(path, "/favicon.ico") == 0) {
        return sprintf(http_response_buffer, "HTTP/1.1 404 Not Found\r\n\r\n");
      }
  }

  if (strcmp(type, "POST") == 0) {
      char command[128];
      char command_response[256];
      strcpy(command, path + 1);
      strcat(command, " ");
      strcat(command, data);
      const size_t command_response_length = process_uart_command(command, command_response);
      const size_t len_pre = sprintf(http_response_buffer, "HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=UTF-8\r\nContent-Length: %"PRIu32"\r\n\r\n", (uint32_t)command_response_length);
      memcpy(http_response_buffer + len_pre, command_response, command_response_length);
      return len_pre + command_response_length;
  }

  return sprintf(http_response_buffer, "HTTP/1.1 404 Not Found\r\n\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
  case LIGHT_SENSOR_D0_Pin:
    GPIO_EXTI_LIGHT_SENSOR_D0();
    break;
  default:
    break;
  }
}

void GPIO_EXTI_LIGHT_SENSOR_D0()
{
  if (htim6.Instance->CR1 & TIM_CR1_CEN) {
    HAL_TIM_Base_Stop_IT(&htim6);
  }

  switch (HAL_GPIO_ReadPin(LIGHT_SENSOR_D0_GPIO_Port, LIGHT_SENSOR_D0_Pin)) {
  case GPIO_PIN_RESET:
    light_sensor_state_requested = -1;
    break;
  case GPIO_PIN_SET:
    light_sensor_state_requested = 1;
    break;
  }

  htim6.Instance->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance != ADC1) {
    return;
  }

  button_update(&sound_sensor,
      HAL_ADC_GetValue(hadc) <= SOUND_SENSOR_THRESHOLD ? RESET : SET,
      HAL_GetTick());
}

void SOUND_SENSOR_Callback(button_state_t old, button_state_t new)
{
  if (old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET) {
    HAL_TIM_Base_Stop_IT(&htim4);
    track_acc += htim4.Instance->CNT;
    track_duration = track_acc;
  } else if (old == BUTTON_SET_REQUESTED && new == BUTTON_SET) {
    if (track_enabled) {
      track_acc = 0;
      htim4.Instance->CNT = 0;
      HAL_TIM_Base_Start_IT(&htim4);
    }
  }
}
/* USER CODE END 4 */

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
