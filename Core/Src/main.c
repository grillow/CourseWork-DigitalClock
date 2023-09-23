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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_A_RESET HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
#define LED_A_SET   HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
#define LED_B_RESET HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
#define LED_B_SET   HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
#define LED_C_RESET HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_RESET);
#define LED_C_SET   HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_SET);
#define LED_D_RESET HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_RESET);
#define LED_D_SET   HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_SET);
#define LED_E_RESET HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_RESET);
#define LED_E_SET   HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_SET);
#define LED_F_RESET HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_RESET);
#define LED_F_SET   HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_SET);
#define LED_G_RESET HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
#define LED_G_SET   HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

#define LED_KEY_1_RESET HAL_GPIO_WritePin(LED_KEY_1_GPIO_Port, LED_KEY_1_Pin, GPIO_PIN_RESET);
#define LED_KEY_1_SET   HAL_GPIO_WritePin(LED_KEY_1_GPIO_Port, LED_KEY_1_Pin, GPIO_PIN_SET);
#define LED_KEY_2_RESET HAL_GPIO_WritePin(LED_KEY_2_GPIO_Port, LED_KEY_2_Pin, GPIO_PIN_RESET);
#define LED_KEY_2_SET   HAL_GPIO_WritePin(LED_KEY_2_GPIO_Port, LED_KEY_2_Pin, GPIO_PIN_SET);
#define LED_KEY_3_RESET HAL_GPIO_WritePin(LED_KEY_3_GPIO_Port, LED_KEY_3_Pin, GPIO_PIN_RESET);
#define LED_KEY_3_SET   HAL_GPIO_WritePin(LED_KEY_3_GPIO_Port, LED_KEY_3_Pin, GPIO_PIN_SET);
#define LED_KEY_4_RESET HAL_GPIO_WritePin(LED_KEY_4_GPIO_Port, LED_KEY_4_Pin, GPIO_PIN_RESET);
#define LED_KEY_4_SET   HAL_GPIO_WritePin(LED_KEY_4_GPIO_Port, LED_KEY_4_Pin, GPIO_PIN_SET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void process_uart_command(UART_HandleTypeDef *huart);
void led_select(uint8_t key);
void led_display(uint8_t number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int uart_rx_buffer_overflow = 0;
char uart_rx_buffer[1];
char command_buffer[32] = {0};
size_t command_buffer_index = 0;

char uart_tx_buffer[256];
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
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // TODO: could probably use timer interruption
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;   // it won't work without reading date
    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
      // failed
      continue;
    }
    const uint32_t LED_DELAY_MS = 1;
    led_select(1);
    led_display(time.Hours / 10);
    HAL_Delay(LED_DELAY_MS);
    led_select(2);
    led_display(time.Hours % 10);
    HAL_Delay(LED_DELAY_MS);
    led_select(3);
    led_display(time.Minutes / 10);
    HAL_Delay(LED_DELAY_MS);
    led_select(4);
    led_display(time.Minutes % 10);
    HAL_Delay(LED_DELAY_MS);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

void process_uart_command(UART_HandleTypeDef *huart) {
  char command[sizeof(command_buffer)] = {0};

  if (sscanf(command_buffer, "%s ", command) != 1) {
    sprintf(uart_tx_buffer, "Error: cannot parse: %s\n", command_buffer);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
    return;
  }

  if (!strcmp(command, "help")) {
    sprintf(uart_tx_buffer, "Commands:\nhelp\nget_time\nset_time <hh> <mm> <ss>\nget_track\n");
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
      sprintf(uart_tx_buffer, "Error: could not get time\n");
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
  } else if (!strcmp(command, "get_track")) {
    //TODO: get_track
    sprintf(uart_tx_buffer, "TODO: %s\n", command);
	  HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  } else {
    sprintf(uart_tx_buffer, "Error: unknown command: %s\n", command);
    HAL_UART_Transmit_IT(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
  }
}

void led_select(uint8_t key) {
  switch (key) {
  default:
    return;
  case 1:
    LED_KEY_1_SET;
    LED_KEY_2_RESET;
    LED_KEY_3_RESET;
    LED_KEY_4_RESET;
    break;
  case 2:
    LED_KEY_1_RESET;
    LED_KEY_2_SET;
    LED_KEY_3_RESET;
    LED_KEY_4_RESET;
    break;
  case 3:
    LED_KEY_1_RESET;
    LED_KEY_2_RESET;
    LED_KEY_3_SET;
    LED_KEY_4_RESET;
    break;
  case 4:
    LED_KEY_1_RESET;
    LED_KEY_2_RESET;
    LED_KEY_3_RESET;
    LED_KEY_4_SET;
    break;
  }
}

void led_display(uint8_t number) {
  number %= 10;

  switch (number) {
  case 0:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_SET;
    break;
  case 1:
    LED_A_SET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  case 2:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_SET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_SET;
    LED_G_RESET;
    break;
  case 3:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_SET;
    LED_G_RESET;
    break;
  case 4:
    LED_A_SET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 5:
    LED_A_RESET;
    LED_B_SET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 6:
    LED_A_RESET;
    LED_B_SET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 7:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  case 8:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 9:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
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
