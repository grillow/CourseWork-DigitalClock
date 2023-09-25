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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

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
/* USER CODE BEGIN PFP */
void process_uart_command(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int uart_rx_buffer_overflow = 0;
char uart_rx_buffer[1];
char command_buffer[32] = {0};
size_t command_buffer_index = 0;
char uart_tx_buffer[256];

// -1 - low, 0 - no change, 1 - high
int light_sensor_state = 0;
int light_sensor_state_requested = 0;
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
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer)) != HAL_OK) {
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
  /* USER CODE END 2 */

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

  /*Configure GPIO pins : LD2_Pin LED_KEY_4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_KEY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  if (huart->Instance != USART2) {
    return;
  }

  const char received_byte = uart_rx_buffer[0];
  switch (received_byte) {
  default:
    if (command_buffer_index >= sizeof(command_buffer)) {
      if (!uart_rx_buffer_overflow) {
        uart_rx_buffer_overflow = 1;
        sprintf(uart_tx_buffer, "Error: command shouldn't be longer than %d\n", sizeof(command_buffer));
          HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
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
    if (!uart_rx_buffer_overflow) {
      process_uart_command(huart);
    }
    uart_rx_buffer_overflow = 0;
    memset(command_buffer, 0, sizeof(command_buffer));
  }
  HAL_UART_Receive_IT(huart, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer));
}

void process_uart_command(UART_HandleTypeDef *huart)
{
  char command[sizeof(command_buffer)] = {0};

  if (sscanf(command_buffer, "%s ", command) != 1) {
    sprintf(uart_tx_buffer, "Error: cannot parse: %s\n", command_buffer);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
    return;
  }

  if (!strcmp(command, "help")) {
    sprintf(uart_tx_buffer, "Commands:\nhelp\nget_time\nset_time <hh> <mm> <ss>\nget_daylight_time\nget_track\n");
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  } else if (!strcmp(command, "get_time")) {
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;   // it won't work without reading date
    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      sprintf(uart_tx_buffer, "Error: could not get time\n");
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    sprintf(uart_tx_buffer, "Time: %02d:%02d:%02d\n", time.Hours, time.Minutes, time.Seconds);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  } else if (!strcmp(command, "set_time")) {
    int hours = 0;
    int minutes = 0;
    int seconds = 0;
    const size_t offset = 9;
    if (sscanf(command_buffer + offset, "%d %d %d", &hours, &minutes, &seconds) != 3) {
      sprintf(uart_tx_buffer, "Error: cannot parse hh mm ss: %s\n", command_buffer + offset);
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    if (hours < 0 || hours >= 24) {
      sprintf(uart_tx_buffer, "Error: wrong hours value: %d\n", hours);
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    if (minutes < 0 || minutes >= 60) {
      sprintf(uart_tx_buffer, "Error: wrong minutes value: %d\n", minutes);
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    if (seconds < 0 || seconds >= 60) {
      sprintf(uart_tx_buffer, "Error: wrong seconds value: %d\n", seconds);
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    // could sscanf directly to this struct, but it would break data validation
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;   // it won't work without reading date
    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      sprintf(uart_tx_buffer, "Error: could not set time\n");
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    time.Hours = hours;
    time.Minutes = minutes;
    time.Seconds = seconds;
    if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      sprintf(uart_tx_buffer, "Error: could not set time\n");
      HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
      return;
    }
    sprintf(uart_tx_buffer, "Time set to %02d:%02d:%02d\n", hours, minutes, seconds);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  } else if (!strcmp(command, "get_daylight_time")) {
    uint32_t light_duration = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_DATA_ADDRESS, 2, (uint8_t*)&light_duration, sizeof(light_duration), HAL_MAX_DELAY) != HAL_OK) {
      //TODO: oy vey (HAL_BUSY)
    }
    const uint32_t seconds = light_duration / 10000;
    const uint32_t minutes = seconds / 60;
    const uint32_t hours   = minutes / 60;
    sprintf(uart_tx_buffer, "Daylight time: %02"PRIu32":%02"PRIu32":%02"PRIu32"\n", hours, minutes % 60, seconds % 60);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  } else if (!strcmp(command, "get_track")) {
    //TODO: get_track
    sprintf(uart_tx_buffer, "TODO: %s\n", command);
	  HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  } else {
    sprintf(uart_tx_buffer, "Error: unknown command: %s\n", command);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != LIGHT_SENSOR_D0_Pin) {
    return;
  }

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
