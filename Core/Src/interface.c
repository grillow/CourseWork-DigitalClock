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
extern TIM_HandleTypeDef htim5;

#define INTERFACE_BUTTON_BOUNCING_TIME_MS 25

static void update_led();
static void led_set_time(led_display_mode_t display_mode, uint32_t hours, uint32_t minutes, uint32_t seconds, uint32_t centiseconds);

static void BUTTON_TOGGLE_DISPLAY_Callback(button_state_t old, button_state_t new);
static void BUTTON_TOGGLE_STOPWATCH_Callback(button_state_t old, button_state_t new);
static void BUTTON_TOGGLE_SET_TIME_Callback(button_state_t old, button_state_t new);
static void BUTTON_DECREASE_Callback(button_state_t old, button_state_t new);
static void BUTTON_INCREASE_Callback(button_state_t old, button_state_t new);


void init_interface()
{
  interface.mode = INTERFACE_DEFAULT_MODE;
  interface.led_display_default_mode = LED_DISPLAY_MODE_HH_MM;
  interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_SS_CS;
  interface.stopwatch_mode.mode = INTERFACE_STOPWATCH_STOPPED;
  interface.stopwatch_mode.stopwatch = 0; //TODO: load?
  interface.led_display_set_time_mode = LED_DISPLAY_MODE_HH;

  interface.BUTTON_TOGGLE_DISPLAY = button_create(
        HAL_GPIO_ReadPin(BUTTON_TOGGLE_DISPLAY_GPIO_Port, BUTTON_TOGGLE_DISPLAY_Pin),
        HAL_GetTick(),
        INTERFACE_BUTTON_BOUNCING_TIME_MS,
        &BUTTON_TOGGLE_DISPLAY_Callback
    );
  interface.BUTTON_TOGGLE_STOPWATCH = button_create(
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_STOPWATCH_GPIO_Port, BUTTON_TOGGLE_STOPWATCH_Pin),
      HAL_GetTick(),
      INTERFACE_BUTTON_BOUNCING_TIME_MS,
      &BUTTON_TOGGLE_STOPWATCH_Callback
  );
  interface.BUTTON_TOGGLE_SET_TIME = button_create(
      HAL_GPIO_ReadPin(BUTTON_TOGGLE_SET_TIME_GPIO_Port, BUTTON_TOGGLE_SET_TIME_Pin),
      HAL_GetTick(),
      INTERFACE_BUTTON_BOUNCING_TIME_MS,
      &BUTTON_TOGGLE_SET_TIME_Callback
  );
  interface.BUTTON_DECREASE = button_create(
      HAL_GPIO_ReadPin(BUTTON_DECREASE_GPIO_Port, BUTTON_DECREASE_Pin),
      HAL_GetTick(),
      INTERFACE_BUTTON_BOUNCING_TIME_MS,
      &BUTTON_DECREASE_Callback
  );
  interface.BUTTON_INCREASE = button_create(
      HAL_GPIO_ReadPin(BUTTON_INCREASE_GPIO_Port, BUTTON_INCREASE_Pin),
      HAL_GetTick(),
      INTERFACE_BUTTON_BOUNCING_TIME_MS,
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

void interface_update_clock()
{
  update_led();
}

void update_led()
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;   // it won't work without reading date
  if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
      HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {}

  switch (interface.mode) {
  case INTERFACE_DEFAULT_MODE:
    led_set_time(interface.led_display_default_mode, time.Hours, time.Minutes, time.Seconds, 0);
    break;
  case INTERFACE_STOPWATCH_MODE: {
      const uint32_t total_centiseconds = htim5.Instance->CNT / 100;
      const uint32_t total_seconds      = total_centiseconds / 100;
      const uint32_t total_minutes      = total_seconds / 60;
      const uint32_t total_hours        = total_minutes / 60;

      const uint32_t centiseconds       = total_centiseconds % 100;
      const uint32_t seconds            = total_seconds % 60;
      const uint32_t minutes            = total_minutes % 60;
      const uint32_t hours              = total_hours;

      led_set_time(interface.led_display_stopwatch_mode, hours, minutes, seconds, centiseconds);
      break;
    }
  case INTERFACE_SET_TIME_MODE: {
      const uint32_t hours   = interface.set_time_mode.hours;
      const uint32_t minutes = interface.set_time_mode.minutes;
      const uint32_t seconds = interface.set_time_mode.seconds;
      led_set_time(interface.led_display_set_time_mode, hours, minutes, seconds, 0);
      break;
    }
  }

  led_out();
}

void led_set_time(led_display_mode_t display_mode, uint32_t hours, uint32_t minutes, uint32_t seconds, uint32_t centiseconds)
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
  case LED_DISPLAY_MODE_SS_CS:
    led_state.led_digit[0] = seconds / 10;
    led_state.led_digit[1] = seconds % 10;
    led_state.led_digit[2] = centiseconds / 10;
    led_state.led_digit[3] = centiseconds % 10;
    break;
  }
}

void BUTTON_TOGGLE_DISPLAY_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.mode) {
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
      interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_SS_CS;
      break;
    case LED_DISPLAY_MODE_MM_SS:
      interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_HH_MM;
      break;
    case LED_DISPLAY_MODE_SS_CS:
      interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_MM_SS;
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

  switch (interface.mode) {
  case INTERFACE_DEFAULT_MODE:
    interface.mode = INTERFACE_STOPWATCH_MODE;
    interface.stopwatch_mode.mode = INTERFACE_STOPWATCH_IDLE;
    break;
  case INTERFACE_STOPWATCH_MODE:
    switch (interface.stopwatch_mode.mode) {
    case INTERFACE_STOPWATCH_IDLE:
      HAL_TIM_Base_Start(&htim5);
      interface.stopwatch_mode.mode = INTERFACE_STOPWATCH_RUNNING;
      interface.led_display_stopwatch_mode = LED_DISPLAY_MODE_SS_CS;
      break;
    case INTERFACE_STOPWATCH_RUNNING:
      interface.stopwatch_mode.mode = INTERFACE_STOPWATCH_STOPPED;
      HAL_TIM_Base_Stop(&htim5);
      break;
    case INTERFACE_STOPWATCH_STOPPED:
      interface.mode = INTERFACE_DEFAULT_MODE;
      interface.stopwatch_mode.stopwatch = htim5.Instance->CNT;
      htim5.Instance->CNT = 0;
      break;
    }
    break;
  case INTERFACE_SET_TIME_MODE:
    // nothing
    break;
  }
}

void BUTTON_TOGGLE_SET_TIME_Callback(button_state_t old, button_state_t new)
{
  if (!(old == BUTTON_RESET_REQUESTED && new == BUTTON_RESET)) {
    return;
  }

  switch (interface.mode) {
  case INTERFACE_DEFAULT_MODE: {
      interface.mode = INTERFACE_SET_TIME_MODE;
      RTC_TimeTypeDef time;
      RTC_DateTypeDef date;   // it won't work without reading date
      if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
          HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {}
      interface.set_time_mode.hours = time.Hours;
      interface.set_time_mode.minutes = time.Minutes;
//      interface.set_time_mode.seconds = time.Seconds;
      interface.set_time_mode.seconds = 0;
    break;
    }
  case INTERFACE_STOPWATCH_MODE:
    // nothing
    break;
  case INTERFACE_SET_TIME_MODE: {
      interface.mode = INTERFACE_DEFAULT_MODE;
      RTC_TimeTypeDef time;
      RTC_DateTypeDef date;
      time.Hours   = interface.set_time_mode.hours;
      time.Minutes = interface.set_time_mode.minutes;
      time.Seconds = interface.set_time_mode.seconds;
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

  switch (interface.mode) {
  case INTERFACE_DEFAULT_MODE:
    // nothing
    break;
  case INTERFACE_STOPWATCH_MODE:
    // nothing
    break;
  case INTERFACE_SET_TIME_MODE:
    switch (interface.led_display_set_time_mode) {
    case LED_DISPLAY_MODE_HH:
      if (interface.set_time_mode.hours-- == 0) {
        interface.set_time_mode.hours = 23;
      }
      break;
    case LED_DISPLAY_MODE_MM:
      if (interface.set_time_mode.minutes-- == 0) {
        interface.set_time_mode.minutes = 59;
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

  switch (interface.mode) {
  case INTERFACE_DEFAULT_MODE:
    // nothing
    break;
  case INTERFACE_STOPWATCH_MODE:
    // nothing
    break;
  case INTERFACE_SET_TIME_MODE:
    switch (interface.led_display_set_time_mode) {
    case LED_DISPLAY_MODE_HH:
      if (++interface.set_time_mode.hours > 23) {
        interface.set_time_mode.hours = 0;
      }
      break;
    case LED_DISPLAY_MODE_MM:
      if (++interface.set_time_mode.minutes > 59) {
        interface.set_time_mode.minutes = 0;
      }
      break;
    default:
      // unreachable
      break;
    }
    break;
  }
}
