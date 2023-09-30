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
  LED_DISPLAY_MODE_HH_MM,
  LED_DISPLAY_MODE_MM_SS,
  LED_DISPLAY_MODE_HH,
  LED_DISPLAY_MODE_MM,
} led_display_mode_t;

typedef struct {
  button_t BUTTON_TOGGLE_DISPLAY;
  led_display_mode_t led_display_mode;
} interface_t;

extern interface_t interface;

void init_interface();
void interface_update_buttons();
void interface_update_led();

#endif /* INC_INTERFACE_H_ */
