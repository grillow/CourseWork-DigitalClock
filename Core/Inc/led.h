/*
 * led.h
 *
 *  Created on: Sep 23, 2023
 *      Author: grillow
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

void led_toggle_display_mode();
void led_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void led_tick();

#endif /* INC_LED_H_ */
