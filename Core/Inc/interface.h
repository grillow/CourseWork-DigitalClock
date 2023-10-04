/*
 * application.h
 *
 *  Created on: Sep 30, 2023
 *      Author: grillow
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include "button.h"
#include "led.h"

typedef enum {
  INTERFACE_DEFAULT_MODE,
  INTERFACE_STOPWATCH_MODE,
  INTERFACE_SET_TIME_MODE,
} interface_mode_t;

typedef enum {
  LED_DISPLAY_MODE_HH_MM,
  LED_DISPLAY_MODE_MM_SS,
  LED_DISPLAY_MODE_HH,
  LED_DISPLAY_MODE_MM,
  LED_DISPLAY_MODE_SS_CS, // seconds, centiseconds
} led_display_mode_t;

typedef struct {
  enum {
    INTERFACE_STOPWATCH_IDLE,
    INTERFACE_STOPWATCH_RUNNING,
    INTERFACE_STOPWATCH_STOPPED,
  } mode;
  uint32_t stopwatch;
} interface_stopwatch_mode_t;

typedef struct {
  uint32_t hours;
  uint32_t minutes;
  uint32_t seconds;
} interface_set_time_mode_t;

typedef struct {
  interface_mode_t mode;
  led_display_mode_t led_display_default_mode;
  led_display_mode_t led_display_stopwatch_mode;
  led_display_mode_t led_display_set_time_mode;
  interface_stopwatch_mode_t stopwatch_mode;
  interface_set_time_mode_t set_time_mode;
  button_t BUTTON_TOGGLE_DISPLAY;
  button_t BUTTON_TOGGLE_STOPWATCH;
  button_t BUTTON_TOGGLE_SET_TIME;
  button_t BUTTON_DECREASE;
  button_t BUTTON_INCREASE;
} interface_t;

extern interface_t interface;

void init_interface();
void interface_update_buttons();
void interface_update_clock();

#endif /* INC_INTERFACE_H_ */
