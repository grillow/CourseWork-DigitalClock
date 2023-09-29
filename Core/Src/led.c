/*
 * led.c
 *
 *  Created on: Sep 23, 2023
 *      Author: grillow
 */

#include "led.h"

static void led_display(uint8_t number);

typedef struct led_state_t {
  enum {
    LED_DISPLAY_HH_MM = 0,
    LED_DISPLAY_MM_SS,
  } led_display_mode;
  uint8_t selected_led;
  uint8_t h1;
  uint8_t h2;
  uint8_t m1;
  uint8_t m2;
  uint8_t s1;
  uint8_t s2;
} led_state_t;

static struct led_state_t led_state = {LED_DISPLAY_HH_MM, 0, 0, 0, 0, 0, 0, 0};

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

void led_toggle_display_mode()
{
  switch (led_state.led_display_mode) {
  case LED_DISPLAY_HH_MM:
    led_state.led_display_mode = LED_DISPLAY_MM_SS;
    break;
  case LED_DISPLAY_MM_SS:
    led_state.led_display_mode = LED_DISPLAY_HH_MM;
    break;
  }
}

void led_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  led_state.h1 = hours / 10;
  led_state.h2 = hours % 10;
  led_state.m1 = minutes / 10;
  led_state.m2 = minutes % 10;
  led_state.s1 = seconds / 10;
  led_state.s2 = seconds % 10;
}

void led_tick()
{
  switch (led_state.selected_led) {
  default:
    return;
  case 0:
    LED_KEY_1_SET;
    LED_KEY_2_RESET;
    LED_KEY_3_RESET;
    LED_KEY_4_RESET;
    switch (led_state.led_display_mode) {
    case LED_DISPLAY_HH_MM:
      led_display(led_state.h1);
      break;
    case LED_DISPLAY_MM_SS:
      led_display(led_state.m1);
      break;
    }
    break;
  case 1:
    LED_KEY_1_RESET;
    LED_KEY_2_SET;
    LED_KEY_3_RESET;
    LED_KEY_4_RESET;
    switch (led_state.led_display_mode) {
    case LED_DISPLAY_HH_MM:
      led_display(led_state.h2);
      break;
    case LED_DISPLAY_MM_SS:
      led_display(led_state.m2);
      break;
    }
    break;
  case 2:
    LED_KEY_1_RESET;
    LED_KEY_2_RESET;
    LED_KEY_3_SET;
    LED_KEY_4_RESET;
    switch (led_state.led_display_mode) {
    case LED_DISPLAY_HH_MM:
      led_display(led_state.m1);
      break;
    case LED_DISPLAY_MM_SS:
      led_display(led_state.s1);
      break;
    }
    break;
  case 3:
    LED_KEY_1_RESET;
    LED_KEY_2_RESET;
    LED_KEY_3_RESET;
    LED_KEY_4_SET;
    switch (led_state.led_display_mode) {
    case LED_DISPLAY_HH_MM:
      led_display(led_state.m2);
      break;
    case LED_DISPLAY_MM_SS:
      led_display(led_state.s2);
      break;
    }
    break;
  }
  if (++led_state.selected_led > 3) {
    led_state.selected_led = 0;
  }
}

void led_display(uint8_t number)
{
  number %= 10;

  switch (number) {
  case 0:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_SET;
    break;
  case 1:
    LED_A_SET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  case 2:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_SET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_SET;
    LED_G_RESET;
    break;
  case 3:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_SET;
    LED_G_RESET;
    break;
  case 4:
    LED_A_SET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 5:
    LED_A_RESET;
    LED_B_SET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 6:
    LED_A_RESET;
    LED_B_SET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 7:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_SET;
    LED_E_SET;
    LED_F_SET;
    LED_G_SET;
    break;
  case 8:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_RESET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  case 9:
    LED_A_RESET;
    LED_B_RESET;
    LED_C_RESET;
    LED_D_RESET;
    LED_E_SET;
    LED_F_RESET;
    LED_G_RESET;
    break;
  }
}
