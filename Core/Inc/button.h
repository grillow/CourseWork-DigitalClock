/*
 * button.h
 *
 *  Created on: Sep 29, 2023
 *      Author: grillow
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

//TODO: GPIO_PinState -> something STM32-independent

typedef enum {
  BUTTON_SET = 1,
  BUTTON_RESET_REQUESTED = -1,
  BUTTON_RESET = 0,
  BUTTON_SET_REQUESTED = 2,
} button_state_t;

typedef struct {
  button_state_t button_state;
  uint32_t state_changed_tick;
  uint32_t bouncing_time;
  void (*callback)(button_state_t old, button_state_t new);
} button_t;

button_t button_create(GPIO_PinState pin, uint32_t current_time, uint32_t bouncing_time,
    void (*callback)(button_state_t, button_state_t));
void button_update(button_t *button, GPIO_PinState pin, uint32_t current_tick);

#endif /* INC_BUTTON_H_ */
