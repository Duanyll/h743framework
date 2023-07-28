#include "led.h"

void LED_Init(void) {
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
  };
#define LED_INIT(led)                                                          \
  s.Pin = LED_##led##_PIN;                                                     \
  HAL_GPIO_Init(LED_##led##_PORT, &s);
  ALL_LEDS(LED_INIT)
}
void LED_On(uint8_t led) {
#define TURN_ON_IF_MATCH(id)                                                   \
  if (led == id) {                                                             \
    HAL_GPIO_WritePin(LED_##id##_PORT, LED_##id##_PIN, GPIO_PIN_SET);          \
    return;                                                                    \
  }
  ALL_LEDS(TURN_ON_IF_MATCH)
}
void LED_Off(uint8_t led) {
#define TURN_OFF_IF_MATCH(id)                                                  \
  if (led == id) {                                                             \
    HAL_GPIO_WritePin(LED_##id##_PORT, LED_##id##_PIN, GPIO_PIN_RESET);        \
    return;                                                                    \
  }
  ALL_LEDS(TURN_OFF_IF_MATCH)
}