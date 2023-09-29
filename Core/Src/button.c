/*
 * button.c
 *
 *  Created on: Sep 29, 2023
 *      Author: grillow
 */

#include "button.h"

button_t create_button(GPIO_PinState pin, uint32_t current_tick, uint32_t bouncing_time,
    void (*callback)(button_state_t, button_state_t)) {
  button_t button;

  switch (pin) {
  case GPIO_PIN_RESET:
    button.button_state = BUTTON_RESET;
    break;
  case GPIO_PIN_SET:
    button.button_state = BUTTON_SET;
    break;
  }
  button.state_changed_tick = current_tick;
  button.bouncing_time = bouncing_time;
  button.callback = callback;

  return button;
}

void update_button(button_t *button, GPIO_PinState pin, uint32_t current_tick) {
  switch (button->button_state) {
  case BUTTON_SET:
    switch (pin) {
      case GPIO_PIN_RESET:
        button->button_state = BUTTON_RESET_REQUESTED;
        button->state_changed_tick = current_tick;
        // no callback
        break;
      case GPIO_PIN_SET:
        // nothing
        break;
      }
    break;
  case BUTTON_RESET_REQUESTED:
    switch (pin) {
      case GPIO_PIN_RESET:
        if (current_tick - button->state_changed_tick > button->bouncing_time) {
          button->button_state = BUTTON_RESET;
          button->state_changed_tick = current_tick;
          button->callback(BUTTON_RESET_REQUESTED, button->button_state);
        }
        break;
      case GPIO_PIN_SET:
        button->button_state = BUTTON_SET;
        button->state_changed_tick = current_tick;
        // no callback
        break;
      }
    break;
  case BUTTON_RESET:
    switch (pin) {
      case GPIO_PIN_RESET:
        // nothing
        break;
      case GPIO_PIN_SET:
        button->button_state = BUTTON_SET_REQUESTED;
        button->state_changed_tick = current_tick;
        // no callback
        break;
      }
    break;
  case BUTTON_SET_REQUESTED:
    switch (pin) {
      case GPIO_PIN_RESET:
        button->button_state = BUTTON_RESET;
        button->state_changed_tick = current_tick;
        // no callback
        break;
      case GPIO_PIN_SET:
        if (current_tick - button->state_changed_tick > button->bouncing_time) {
          button->button_state = BUTTON_SET;
          button->state_changed_tick = current_tick;
          button->callback(BUTTON_SET_REQUESTED, button->button_state);
        }
        break;
      }
    break;
  }
}
