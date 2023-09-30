/*
 * application.c
 *
 *  Created on: Sep 30, 2023
 *      Author: grillow
 */

#include <interface.h>
#include "main.h"

interface_t interface;
extern RTC_HandleTypeDef hrtc;

#define BUTTON_BOUNCING_TIME_MS 50

static void BUTTON_TOGGLE_DISPLAY_Callback(button_state_t old, button_state_t new);

void init_interface()
{
  interface.BUTTON_TOGGLE_DISPLAY = button_create(
        HAL_GPIO_ReadPin(BUTTON_TOGGLE_DISPLAY_GPIO_Port, BUTTON_TOGGLE_DISPLAY_Pin),
        HAL_GetTick(),
        BUTTON_BOUNCING_TIME_MS,
        &BUTTON_TOGGLE_DISPLAY_Callback
    );
  interface.led_display_mode = LED_DISPLAY_MODE_HH_MM;
}

void interface_update_buttons()
{
  button_update(&interface.BUTTON_TOGGLE_DISPLAY,
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_DISPLAY_GPIO_Port, BUTTON_TOGGLE_DISPLAY_Pin),
      HAL_GetTick());
}

void interface_update_led()
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;   // it won't work without reading date
  if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
      HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
    return;
  }

  switch (interface.led_display_mode) {
  case LED_DISPLAY_MODE_HH_MM:
    led_state.led_digit[0] = time.Hours / 10;
    led_state.led_digit[1] = time.Hours % 10;
    led_state.led_digit[2] = time.Minutes / 10;
    led_state.led_digit[3] = time.Minutes % 10;
    break;
  case LED_DISPLAY_MODE_MM_SS:
    led_state.led_digit[0] = time.Minutes / 10;
    led_state.led_digit[1] = time.Minutes % 10;
    led_state.led_digit[2] = time.Seconds / 10;
    led_state.led_digit[3] = time.Seconds % 10;
    break;
  case LED_DISPLAY_MODE_HH:
    led_state.led_digit[0] = time.Hours / 10;
    led_state.led_digit[1] = time.Hours % 10;
    led_state.led_digit[2] = LED_DIGIT_NONE;
    led_state.led_digit[3] = LED_DIGIT_NONE;
    break;
  case LED_DISPLAY_MODE_MM:
    led_state.led_digit[0] = LED_DIGIT_NONE;
    led_state.led_digit[1] = LED_DIGIT_NONE;
    led_state.led_digit[2] = time.Minutes / 10;
    led_state.led_digit[3] = time.Minutes % 10;
    break;
  }

  led_out();
}

void BUTTON_TOGGLE_DISPLAY_Callback(button_state_t old, button_state_t new)
{
  if (old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET) {
    switch (interface.led_display_mode) {
    case LED_DISPLAY_MODE_HH_MM:
      interface.led_display_mode = LED_DISPLAY_MODE_MM_SS;
      break;
    case LED_DISPLAY_MODE_MM_SS:
      interface.led_display_mode = LED_DISPLAY_MODE_HH_MM;
      break;
    }
  }
}
