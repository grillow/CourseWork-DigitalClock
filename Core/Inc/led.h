/*
 * led.h
 *
 *  Created on: Sep 23, 2023
 *      Author: grillow
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

typedef enum {
  LED_DIGIT_ZERO  = 0,
  LED_DIGIT_ONE   = 1,
  LED_DIGIT_TWO   = 2,
  LED_DIGIT_THREE = 3,
  LED_DIGIT_FOUR  = 4,
  LED_DIGIT_FIVE  = 5,
  LED_DIGIT_SIX   = 6,
  LED_DIGIT_SEVEN = 7,
  LED_DIGIT_EIGHT = 8,
  LED_DIGIT_NINE  = 9,
  LED_DIGIT_NONE  = 10
} led_digit_t;

typedef struct {
  uint8_t selected_led;
  led_digit_t led_digit[4];
} led_state_t;

extern led_state_t led_state;

void led_out();

#endif /* INC_LED_H_ */
