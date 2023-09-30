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

static void led_set_time(led_display_mode_t display_mode, uint32_t hours, uint32_t minutes, uint32_t seconds);

static void BUTTON_TOGGLE_DISPLAY_Callback(button_state_t old, button_state_t new);
static void BUTTON_TOGGLE_STOPWATCH_Callback(button_state_t old, button_state_t new);
static void BUTTON_TOGGLE_SET_TIME_Callback(button_state_t old, button_state_t new);
static void BUTTON_DECREASE_Callback(button_state_t old, button_state_t new);
static void BUTTON_INCREASE_Callback(button_state_t old, button_state_t new);


void init_interface()
{
  interface.interface_mode = INTERFACE_DEFAULT_MODE;
  interface.led_display_default_mode = LED_DISPLAY_MODE_HH_MM;
  interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_MM_SS;
  interface.led_display_set_time_mode = LED_DISPLAY_MODE_HH;

  interface.BUTTON_TOGGLE_DISPLAY = button_create(
        HAL_GPIO_ReadPin(BUTTON_TOGGLE_DISPLAY_GPIO_Port, BUTTON_TOGGLE_DISPLAY_Pin),
        HAL_GetTick(),
        BUTTON_BOUNCING_TIME_MS,
        &BUTTON_TOGGLE_DISPLAY_Callback
    );
  interface.BUTTON_TOGGLE_STOPWATCH = button_create(
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_STOPWATCH_GPIO_Port, BUTTON_TOGGLE_STOPWATCH_Pin),
      HAL_GetTick(),
      BUTTON_BOUNCING_TIME_MS,
      &BUTTON_TOGGLE_STOPWATCH_Callback
  );
  interface.BUTTON_TOGGLE_SET_TIME = button_create(
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_SET_TIME_GPIO_Port, BUTTON_TOGGLE_SET_TIME_Pin),
      HAL_GetTick(),
      BUTTON_BOUNCING_TIME_MS,
      &BUTTON_TOGGLE_SET_TIME_Callback
  );
  interface.BUTTON_DECREASE = button_create(
      HAL_GPIO_ReadPin(BUTTON_DECREASE_GPIO_Port, BUTTON_DECREASE_Pin),
      HAL_GetTick(),
      BUTTON_BOUNCING_TIME_MS,
      &BUTTON_DECREASE_Callback
  );
  interface.BUTTON_INCREASE = button_create(
      HAL_GPIO_ReadPin(BUTTON_INCREASE_GPIO_Port, BUTTON_INCREASE_Pin),
      HAL_GetTick(),
      BUTTON_BOUNCING_TIME_MS,
      &BUTTON_INCREASE_Callback
  );
}

void interface_update_buttons()
{
  button_update(&interface.BUTTON_TOGGLE_DISPLAY,
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_DISPLAY_GPIO_Port, BUTTON_TOGGLE_DISPLAY_Pin),
      HAL_GetTick());
  button_update(&interface.BUTTON_TOGGLE_STOPWATCH,
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_STOPWATCH_GPIO_Port, BUTTON_TOGGLE_STOPWATCH_Pin),
      HAL_GetTick());
  button_update(&interface.BUTTON_TOGGLE_SET_TIME,
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_SET_TIME_GPIO_Port, BUTTON_TOGGLE_SET_TIME_Pin),
      HAL_GetTick());
  button_update(&interface.BUTTON_DECREASE,
      HAL_GPIO_ReadPin(BUTTON_DECREASE_GPIO_Port, BUTTON_DECREASE_Pin),
      HAL_GetTick());
  button_update(&interface.BUTTON_INCREASE,
      HAL_GPIO_ReadPin(BUTTON_INCREASE_GPIO_Port, BUTTON_INCREASE_Pin),
      HAL_GetTick());
}

void interface_update_led()
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;   // it won't work without reading date
  if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
      HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {}

  switch (interface.interface_mode) {
  case INTERFACE_DEFAULT_MODE:
    led_set_time(interface.led_display_default_mode, time.Hours, time.Minutes, time.Seconds);
    break;
  case INTERFACE_STOPWATCH_MODE:
    //TODO:
    led_set_time(interface.led_display_stopwatch_mode, 69, 13, 37);
    break;
  case INTERFACE_SET_TIME_MODE: {
      const uint32_t hours   = interface.interface_set_time_mode.hours;
      const uint32_t minutes = interface.interface_set_time_mode.minutes;
      const uint32_t seconds = interface.interface_set_time_mode.seconds;
      led_set_time(interface.led_display_set_time_mode, hours, minutes, seconds);
      break;
    }
  }

  led_out();
}

void led_set_time(led_display_mode_t display_mode, uint32_t hours, uint32_t minutes, uint32_t seconds)
{
  switch (display_mode) {
  case LED_DISPLAY_MODE_HH_MM:
    led_state.led_digit[0] = hours / 10;
    led_state.led_digit[1] = hours % 10;
    led_state.led_digit[2] = minutes / 10;
    led_state.led_digit[3] = minutes % 10;
    break;
  case LED_DISPLAY_MODE_MM_SS:
    led_state.led_digit[0] = minutes / 10;
    led_state.led_digit[1] = minutes % 10;
    led_state.led_digit[2] = seconds / 10;
    led_state.led_digit[3] = seconds % 10;
    break;
  case LED_DISPLAY_MODE_HH:
    led_state.led_digit[0] = hours / 10;
    led_state.led_digit[1] = hours % 10;
    led_state.led_digit[2] = LED_DIGIT_NONE;
    led_state.led_digit[3] = LED_DIGIT_NONE;
    break;
  case LED_DISPLAY_MODE_MM:
    led_state.led_digit[0] = LED_DIGIT_NONE;
    led_state.led_digit[1] = LED_DIGIT_NONE;
    led_state.led_digit[2] = minutes / 10;
    led_state.led_digit[3] = minutes % 10;
    break;
  }
}

void BUTTON_TOGGLE_DISPLAY_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.interface_mode) {
  case INTERFACE_DEFAULT_MODE:
    switch (interface.led_display_default_mode) {
    case LED_DISPLAY_MODE_HH_MM:
      interface.led_display_default_mode = LED_DISPLAY_MODE_MM_SS;
      break;
    case LED_DISPLAY_MODE_MM_SS:
      interface.led_display_default_mode = LED_DISPLAY_MODE_HH_MM;
      break;
    default:
      // unreachable
    break;
    }
    break;
  case INTERFACE_STOPWATCH_MODE:
    switch (interface.led_display_stopwatch_mode) {
    case LED_DISPLAY_MODE_HH_MM:
      interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_MM_SS;
      break;
    case LED_DISPLAY_MODE_MM_SS:
      interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_HH_MM;
      break;
    default:
      // unreachable
      break;
    }
    break;
  case INTERFACE_SET_TIME_MODE:
    switch (interface.led_display_set_time_mode) {
    case LED_DISPLAY_MODE_HH:
      interface.led_display_set_time_mode = LED_DISPLAY_MODE_MM;
      break;
    case LED_DISPLAY_MODE_MM:
      interface.led_display_set_time_mode = LED_DISPLAY_MODE_HH;
      break;
    default:
      // unreachable
      break;
    }
    break;
  }
}

void BUTTON_TOGGLE_STOPWATCH_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.interface_mode) {
  case INTERFACE_DEFAULT_MODE:
    interface.interface_mode = INTERFACE_STOPWATCH_MODE;
    interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_MM_SS;
    break;
  case INTERFACE_STOPWATCH_MODE:
    interface.interface_mode = INTERFACE_DEFAULT_MODE;

    break;
  case INTERFACE_SET_TIME_MODE:
    // nothing
    break;
  }

  //TODO:
}

void BUTTON_TOGGLE_SET_TIME_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.interface_mode) {
  case INTERFACE_DEFAULT_MODE: {
      interface.interface_mode = INTERFACE_SET_TIME_MODE;
      RTC_TimeTypeDef time;
      RTC_DateTypeDef date;   // it won't work without reading date
      if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
          HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {}
      interface.interface_set_time_mode.hours = time.Hours;
      interface.interface_set_time_mode.minutes = time.Minutes;
      interface.interface_set_time_mode.seconds = time.Seconds;
    break;
    }
  case INTERFACE_STOPWATCH_MODE:
    // nothing
    break;
  case INTERFACE_SET_TIME_MODE: {
      interface.interface_mode = INTERFACE_DEFAULT_MODE;
      RTC_TimeTypeDef time;
      RTC_DateTypeDef date;
      time.Hours   = interface.interface_set_time_mode.hours;
      time.Minutes = interface.interface_set_time_mode.minutes;
      time.Seconds = interface.interface_set_time_mode.seconds;
      if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
          HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {}
      break;
    }
  }
}

void BUTTON_DECREASE_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.interface_mode) {
  case INTERFACE_DEFAULT_MODE:
    // nothing
    break;
  case INTERFACE_STOPWATCH_MODE:
    // nothing
    break;
  case INTERFACE_SET_TIME_MODE:
    switch (interface.led_display_set_time_mode) {
    case LED_DISPLAY_MODE_HH:
      if (interface.interface_set_time_mode.hours-- == 0) {
        interface.interface_set_time_mode.hours = 23;
      }
      break;
    case LED_DISPLAY_MODE_MM:
      if (interface.interface_set_time_mode.minutes-- == 0) {
        interface.interface_set_time_mode.minutes = 59;
      }
      break;
    default:
      // unreachable
      break;
    }
    break;
  }
}

void BUTTON_INCREASE_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.interface_mode) {
  case INTERFACE_DEFAULT_MODE:
    // nothing
    break;
  case INTERFACE_STOPWATCH_MODE:
    // nothing
    break;
  case INTERFACE_SET_TIME_MODE:
    switch (interface.led_display_set_time_mode) {
    case LED_DISPLAY_MODE_HH:
      if (++interface.interface_set_time_mode.hours > 23) {
        interface.interface_set_time_mode.hours = 0;
      }
      break;
    case LED_DISPLAY_MODE_MM:
      if (++interface.interface_set_time_mode.minutes > 59) {
        interface.interface_set_time_mode.minutes = 0;
      }
      break;
    default:
      // unreachable
      break;
    }
    break;
  }
}
