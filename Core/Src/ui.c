#include "ui.h"

#include "zjy122250/bmp.h"

static EPD_Pins EPD_pins;
static uint8_t image_bw[EPD_W_BUFF_SIZE * EPD_H];
static char text[20];

void UI_InitEPD(void) {
  EPD_pins.BUSY_Port = GPIOB;
  EPD_pins.BUSY_Pin = GPIO_PIN_7;
  EPD_pins.RES_Port = GPIOB;
  EPD_pins.RES_Pin = GPIO_PIN_8;
  EPD_pins.DC_Port = GPIOB;
  EPD_pins.DC_Pin = GPIO_PIN_9;
  EPD_pins.CS_Port = GPIOB;
  EPD_pins.CS_Pin = GPIO_PIN_10;
  EPD_pins.SCL_Port = GPIOB;
  EPD_pins.SCL_Pin = GPIO_PIN_11;
  EPD_pins.SDA_Port = GPIOB;
  EPD_pins.SDA_Pin = GPIO_PIN_12;

  GPIO_InitTypeDef s = {
    .Pin = GPIO_PIN_0,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(GPIOB, &s);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(100);

  EPD_io_init(&EPD_pins);
  EPD_init();
}
void UI_TestEPD(void) {
  EPD_paint_newimage(image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);

  EPD_paint_selectimage(image_bw);
  EPD_paint_clear(EPD_COLOR_WHITE);
  EPD_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122,
                        gImage_4, EPD_COLOR_WHITE);

  EPD_displayBW(image_bw);
  EPD_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

  HAL_Delay(5000);

  EPD_init_partial();

  EPD_paint_selectimage(image_bw);
  EPD_paint_clear(EPD_COLOR_WHITE);

  EPD_paint_showString(10, 0, (uint8_t *)&"2.13 Epaper Module",
                       EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
  EPD_paint_showString(10, 50, (uint8_t *)&"with 250 x 122 resolution",
                       EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

  EPD_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio",
                       EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

  EPD_paint_drawRectangle(10, 103, EPD_H - 10, 116, EPD_COLOR_BLACK, 1);

  sprintf((char *)&text, ">> Partial Mode");
  EPD_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

  EPD_displayBW_partial(image_bw);

  HAL_Delay(1000);

  for (uint32_t i = 123; i < 8 * 123; i += 123) {
    sprintf((char *)&text, ">> Num=%d     ", i);
    EPD_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    EPD_displayBW_partial(image_bw);

    HAL_Delay(100);
  }

  sprintf((char *)&text, ">> Hello World.");
  EPD_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
  EPD_displayBW_partial(image_bw);

  HAL_Delay(1000);

  EPD_update();
}

#define UI_TERMINAL_HEIGHT 10
#define UI_TERMINAL_WIDTH 35

static char terminal[UI_TERMINAL_HEIGHT][UI_TERMINAL_WIDTH]; // Circular buffer for history
static uint8_t terminal_head = 0;
static uint8_t terminal_tail = 0;

void UI_TerminalInit() {
  terminal_head = terminal_tail = 0;
  EPD_paint_newimage(image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
  EPD_paint_selectimage(image_bw);
  EPD_paint_clear(EPD_COLOR_WHITE);
  EPD_displayBW(image_bw);
  HAL_Delay(100);
  EPD_init_partial();
}

void UI_TerminalRender() {
  EPD_paint_selectimage(image_bw);
  EPD_paint_clear(EPD_COLOR_WHITE);
  int line = terminal_tail;
  int idx = 0;
  while (line != terminal_head) {
    EPD_paint_showString(5, idx * 12 + 1, terminal[line], EPD_FONT_SIZE12x6, EPD_COLOR_BLACK);
    line = (line + 1) % UI_TERMINAL_HEIGHT;
    idx++;
  }
  EPD_displayBW_partial(image_bw);
}

void UI_TerminalPrintf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(terminal[terminal_head], UI_TERMINAL_WIDTH, format, args);
  va_end(args);
  terminal_head = (terminal_head + 1) % UI_TERMINAL_HEIGHT;
  if (terminal_head == terminal_tail) {
    terminal_tail = (terminal_tail + 1) % UI_TERMINAL_HEIGHT;
  }
  UI_TerminalRender();
}

void UI_TerminalFlush() {
  EPD_update();
}

void UI_TerminalClear() {
  terminal_head = terminal_tail = 0;
  UI_TerminalRender();
}

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t w;
  uint8_t h;
  float *data;
  int len;
  int min;
  int max;
  int stride;
  
} UI_PlotOptionsF32;

void UI_Plot(UI_PlotOptionsF32 *opt) {
  
}