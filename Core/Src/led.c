/*
 * led.c
 *
 *  Created on: Sep 23, 2023
 *      Author: grillow
 */

#include <led.h>

static void led_out_digit(uint8_t digit);

led_state_t led_state = {0, {1, 3, 3, 7}};

#define LED_A_RESET HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
#define LED_A_SET   HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
#define LED_B_RESET HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
#define LED_B_SET   HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
#define LED_C_RESET HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_RESET);
#define LED_C_SET   HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_SET);
#define LED_D_RESET HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_RESET);
#define LED_D_SET   HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_SET);
#define LED_E_RESET HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_RESET);
#define LED_E_SET   HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_SET);
#define LED_F_RESET HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_RESET);
#define LED_F_SET   HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_SET);
#define LED_G_RESET HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
#define LED_G_SET   HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

#define LED_KEY_1_RESET HAL_GPIO_WritePin(LED_KEY_1_GPIO_Port, LED_KEY_1_Pin, GPIO_PIN_RESET);
#define LED_KEY_1_SET   HAL_GPIO_WritePin(LED_KEY_1_GPIO_Port, LED_KEY_1_Pin, GPIO_PIN_SET);
#define LED_KEY_2_RESET HAL_GPIO_WritePin(LED_KEY_2_GPIO_Port, LED_KEY_2_Pin, GPIO_PIN_RESET);
#define LED_KEY_2_SET   HAL_GPIO_WritePin(LED_KEY_2_GPIO_Port, LED_KEY_2_Pin, GPIO_PIN_SET);
#define LED_KEY_3_RESET HAL_GPIO_WritePin(LED_KEY_3_GPIO_Port, LED_KEY_3_Pin, GPIO_PIN_RESET);
#define LED_KEY_3_SET   HAL_GPIO_WritePin(LED_KEY_3_GPIO_Port, LED_KEY_3_Pin, GPIO_PIN_SET);
#define LED_KEY_4_RESET HAL_GPIO_WritePin(LED_KEY_4_GPIO_Port, LED_KEY_4_Pin, GPIO_PIN_RESET);
#define LED_KEY_4_SET   HAL_GPIO_WritePin(LED_KEY_4_GPIO_Port, LED_KEY_4_Pin, GPIO_PIN_SET);


void led_out()
{
  switch (led_state.selected_led) {
  default:
    return;
  case 0:
    LED_KEY_1_SET;
    LED_KEY_2_RESET;
    LED_KEY_3_RESET;
    LED_KEY_4_RESET;
    break;
  case 1:
    LED_KEY_1_RESET;
    LED_KEY_2_SET;
    LED_KEY_3_RESET;
    LED_KEY_4_RESET;
    break;
  case 2:
    LED_KEY_1_RESET;
    LED_KEY_2_RESET;
    LED_KEY_3_SET;
    LED_KEY_4_RESET;
    break;
  case 3:
    LED_KEY_1_RESET;
    LED_KEY_2_RESET;
    LED_KEY_3_RESET;
    LED_KEY_4_SET;
    break;
  }
  led_out_digit(led_state.led_digit[led_state.selected_led]);
  if (++led_state.selected_led > 3) {
    led_state.selected_led = 0;
  }
}

void led_out_digit(led_digit_t digit)
{
  switch (digit) {
  case LED_DIGIT_ZERO:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_SET;
    break;
  case LED_DIGIT_ONE:
    LED_A_SET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  case LED_DIGIT_TWO:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_SET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_SET;
    LED_G_RESET;
    break;
  case LED_DIGIT_THREE:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_SET;
    LED_G_RESET;
    break;
  case LED_DIGIT_FOUR:
    LED_A_SET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case LED_DIGIT_FIVE:
    LED_A_RESET;
    LED_B_SET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case LED_DIGIT_SIX:
    LED_A_RESET;
    LED_B_SET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case LED_DIGIT_SEVEN:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  case LED_DIGIT_EIGHT:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case LED_DIGIT_NINE:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case LED_DIGIT_NONE:
    LED_A_SET;
    LED_B_SET;
    LED_C_SET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  }
}
