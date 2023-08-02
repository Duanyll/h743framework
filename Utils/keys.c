#include "keys.h"
#include "timers.h"

static KEYS_Pins *p;

static struct {
  int holdCount;
  BOOL isOn;
  int releaseCount;
} state[KEYS_MAX_KEYS];

typedef struct {
  uint8_t key;
  uint8_t event;
} KEYS_Event;
static KEYS_Event eventQueue[KEYS_EVENT_QUEUE_SIZE];
static int eventQueueHead = 0;
static int eventQueueTail = 0;

void KEYS_Init(KEYS_Pins *pins) {
  p = pins;
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_INPUT,
      .Pull = GPIO_PULLUP,
      .Speed = GPIO_SPEED_FREQ_LOW,
  };
  for (int i = 0; i < p->keyCount; i++) {
    s.Pin = p->pins[i].pin;
    HAL_GPIO_Init(p->pins[i].port, &s);
    state[i].holdCount = 0;
    state[i].isOn = FALSE;
    state[i].releaseCount = 0;
  }
}

void KEYS_PushEvent(uint8_t key, uint8_t event) {
  if ((eventQueueHead + 1) % KEYS_EVENT_QUEUE_SIZE == eventQueueTail) {
    return;
  }
  eventQueue[eventQueueHead].key = key;
  eventQueue[eventQueueHead].event = event;
  eventQueueHead = (eventQueueHead + 1) % KEYS_EVENT_QUEUE_SIZE;
}

void KEYS_TimerCallback() {
  for (int i = 0; i < p->keyCount; i++) {
    BOOL isOn =
        HAL_GPIO_ReadPin(p->pins[i].port, p->pins[i].pin) == GPIO_PIN_RESET;
    if (state[i].isOn) {
      if (isOn) {
        state[i].holdCount++;
        state[i].releaseCount = 0;
        if (state[i].holdCount == KEYS_LONG_HOLD_COUNT) {
          KEYS_PushEvent(i, KEYS_EVENT_HOLD);
          // printf("KEYS_EVENT_HOLD\n");
        }
      } else {
        state[i].releaseCount++;
        if (state[i].releaseCount >= KEYS_HOLD_COUNT) {
          state[i].isOn = FALSE;
          state[i].holdCount = 0;
          state[i].releaseCount = 0;
          KEYS_PushEvent(i, KEYS_EVENT_RELEASE);
          // printf("KEYS_EVENT_RELEASE\n");
        }
      }
    } else {
      if (isOn) {
        state[i].holdCount++;
        if (state[i].holdCount >= KEYS_HOLD_COUNT) {
          state[i].isOn = TRUE;
          KEYS_PushEvent(i, KEYS_EVENT_PRESS);
          // printf("KEYS_EVENT_PRESS\n");
        }
      } else {
        state[i].holdCount = 0;
      }
    }
  }
}

void KEYS_Start() {
  eventQueueHead = 0;
  eventQueueTail = 0;
  TIM_RegisterCallback(p->htim, KEYS_TimerCallback);
  TIM_StartPeriodic(p->htim, KEYS_SAMPLE_RATE);
}
void KEYS_Stop() { TIM_StopPeriodic(p->htim); }
void KEYS_Poll() {
  while (eventQueueHead != eventQueueTail) {
    KEYS_Event *e = &eventQueue[eventQueueTail];
    if (p->pins[e->key].callback != NULL) {
      p->pins[e->key].callback(e->event);
    }
    eventQueueTail = (eventQueueTail + 1) % KEYS_EVENT_QUEUE_SIZE;
  }
}