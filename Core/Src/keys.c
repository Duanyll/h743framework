#include "keys.h"
#include "main.h"

uint16_t keys_pin[KEYS_COUNT];
GPIO_TypeDef* keys_port;

uint32_t keys_update_time[KEYS_COUNT];
uint32_t keys_pressed_time[KEYS_COUNT];
uint8_t keys_state[KEYS_COUNT];

void (*keys_handler)(uint8_t key, uint8_t state);
static void KEYS_Handler(uint8_t key, uint8_t state) {
    if (keys_handler != NULL) {
        keys_handler(key, state);
    }
}

void KEYS_Init() {
    keys_pin[0] = SWITCH1_Pin;
    keys_pin[1] = SWITCH2_Pin;
    keys_pin[2] = SWITCH3_Pin;
    keys_pin[3] = SWITCH4_Pin;
    keys_port = SWITCH1_GPIO_Port;
}
void KEYS_Scan() {
    for (int i = 0; i < KEYS_COUNT; i++) {
        int state = HAL_GPIO_ReadPin(keys_port, keys_pin[i]);
        int time = HAL_GetTick();
        int dt = time - keys_update_time[i];
        keys_update_time[i] = time;
        if (state == GPIO_PIN_RESET) {
            keys_pressed_time[i] += dt;
            if (keys_state[i] == KEYS_STATE_RELEASE && keys_pressed_time[i] > KEYS_DEBOUNCE_TIME) {
                keys_state[i] = KEYS_STATE_PRESS;
                KEYS_Handler(i, keys_state[i]);
            } else if (keys_state[i] == KEYS_STATE_PRESS && keys_pressed_time[i] > KEYS_LONG_PRESS_TIME) {
                keys_state[i] = KEYS_STATE_LONG_PRESS;
                KEYS_Handler(i, keys_state[i]);
            }
        } else {
            keys_pressed_time[i] = 0;
            if (keys_state[i] == KEYS_STATE_PRESS || keys_state[i] == KEYS_STATE_LONG_PRESS) {
                keys_state[i] = KEYS_STATE_RELEASE;
                KEYS_Handler(i, keys_state[i]);
            }
        }
    }
}
uint8_t KEYS_GetState(uint8_t key) {
    return keys_state[key];
}
void KEYS_SetHandler(void (*handler)(uint8_t key, uint8_t state)) {
    keys_handler = handler;
}