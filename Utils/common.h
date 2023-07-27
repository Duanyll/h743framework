#include "main.h"

#define BOOL uint8_t
#define TRUE 1
#define FALSE 0

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, a, b) MIN(MAX((x), (a)), (b))

#define M_PI 3.14159265358979323846

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET