#define KEYS_MAX_KEYS 8
#define KEYS_SAMPLE_RATE 100
#define KEYS_HOLD_COUNT 10
#define KEYS_LONG_HOLD_COUNT 100

#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024

#define ALL_UART_PORTS(_)                                                      \
  _(USART1)                                                                    \
  _(USART2)                                                                    \
  _(USART3)                                                                    \
  _(USART6)

#define ALL_TIMERS(_)                                                          \
  _(TIM2)                                                                      \
  _(TIM3)                                                                      \
  _(TIM4)                                                                      \
  _(TIM5)                                                                      \
  _(TIM6)                                                                      \
  _(TIM7)

#define SIGNAL_MAX_PEAKS 32

// #define LED_1_PORT GPIOE
// #define LED_1_PIN GPIO_PIN_4
// #define LED_2_PORT GPIOE
// #define LED_2_PIN GPIO_PIN_5
// #define LED_3_PORT GPIOE
// #define LED_3_PIN GPIO_PIN_6

#define LED_1_PORT GPIOE
#define LED_1_PIN GPIO_PIN_13
#define LED_2_PORT GPIOE
#define LED_2_PIN GPIO_PIN_14
#define LED_3_PORT GPIOE
#define LED_3_PIN GPIO_PIN_15

#define ALL_LEDS(_)                                                            \
  _(1)                                                                         \
  _(2)                                                                         \
  _(3)
