/*
 * led.h
 *
 *  Created on: Sep 23, 2023
 *      Author: grillow
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

#include "main.h"

typedef enum {
  CLOCK_DISPLAY_MODE_HH_MM,
  CLOCK_DISPLAY_MODE_MM_SS,
} clock_display_mode_t;

typedef struct {
  clock_display_mode_t display_mode;
  uint8_t selected_led;
} clock_state_t;

extern clock_state_t clock_state;

void clock_display(uint8_t hours, uint8_t minutes, uint8_t seconds);

#endif /* INC_CLOCK_H_ */
