#ifndef __EPAPER_H
#define __EPAPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

typedef struct {
  GPIO_TypeDef* BUSY_Port;
  uint16_t BUSY_Pin;
  GPIO_TypeDef* RES_Port;
  uint16_t RES_Pin;
  GPIO_TypeDef* DC_Port;
  uint16_t DC_Pin;
  GPIO_TypeDef* CS_Port;
  uint16_t CS_Pin;
  GPIO_TypeDef* SCL_Port;
  uint16_t SCL_Pin;
  GPIO_TypeDef* SDA_Port;
  uint16_t SDA_Pin;
} EPD_Pins;

// #define EPD_29
#define EPD_213

#if (!defined EPD_29) && (!defined EPD_213)
#error EPD Type Undefine
#endif

#ifdef EPD_29
#define EPD_W 128
#define EPD_H 296
#endif

#ifdef EPD_213
#define EPD_W 122
#define EPD_H 250
#endif

#define EPD_OK 0
#define EPD_ERROR 1

#define EPD_ROTATE_0 0
#define EPD_ROTATE_90 90
#define EPD_ROTATE_180 180
#define EPD_ROTATE_270 270

#define EPD_COLOR_WHITE 0xFF
#define EPD_COLOR_BLACK 0x00
#define EPD_COLOR_RED EPD_COLOR_BLACK

#define EPD_FONT_SIZE8x6 (8)
#define EPD_FONT_SIZE12x6 (12)
#define EPD_FONT_SIZE16x8 (16)
#define EPD_FONT_SIZE24x12 (24)

#define EPD_DEEPSLEEP_MODE1 (0x01)
#define EPD_DEEPSLEEP_MODE2 (0x03)

#define EPD_W_BUFF_SIZE ((EPD_W % 8 == 0) ? (EPD_W / 8) : (EPD_W / 8 + 1))
typedef struct {
  uint8_t *Image;
  uint16_t Width;
  uint16_t Height;
  uint16_t WidthMemory;
  uint16_t HeightMemory;
  uint16_t Color;
  uint16_t Rotate;
  uint16_t WidthByte;
  uint16_t HeightByte;
} EPD_PAINT;
extern EPD_PAINT EPD_Paint;

void EPD_io_init(EPD_Pins *pins);
uint8_t EPD_init(void);
uint8_t EPD_init_partial(void);
void EPD_enter_deepsleepmode(uint8_t mode);
void EPD_init_internalTempSensor(void);
void EPD_update(void);
void EPD_update_partial(void);
void EPD_display(uint8_t *Image1, uint8_t *Image2);
void EPD_displayBW(uint8_t *Image);
void EPD_displayBW_partial(uint8_t *Image);
// void EPD_displayRED(uint8_t *Image);

void EPD_paint_newimage(uint8_t *image, uint16_t Width, uint16_t Height,
                        uint16_t Rotate, uint16_t Color);
void EPD_paint_setpixel(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color);
void EPD_paint_selectimage(uint8_t *image);
void EPD_paint_clear(uint16_t color);
void EPD_paint_drawPoint(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color);
void EPD_paint_drawLine(uint16_t Xstart, uint16_t Ystart, uint16_t Xend,
                        uint16_t Yend, uint16_t Color);
void EPD_paint_drawRectangle(uint16_t Xstart, uint16_t Ystart, uint16_t Xend,
                             uint16_t Yend, uint16_t Color, uint8_t mode);
void EPD_paint_drawCircle(uint16_t X_Center, uint16_t Y_Center, uint16_t Radius,
                          uint16_t Color, uint8_t mode);
void EPD_paint_showChar(uint16_t x, uint16_t y, uint16_t chr, uint16_t size1,
                        uint16_t color);
void EPD_paint_showString(uint16_t x, uint16_t y, char *chr, uint16_t size1,
                          uint16_t color);
void EPD_paint_showNum(uint16_t x, uint16_t y, uint32_t num, uint16_t len,
                       uint16_t size1, uint16_t color);
void EPD_paint_showChinese(uint16_t x, uint16_t y, uint16_t num, uint16_t size1,
                           uint16_t color);
void EPD_paint_showPicture(uint16_t x, uint16_t y, uint16_t sizex,
                           uint16_t sizey, const uint8_t BMP[], uint16_t Color);

#ifdef __cplusplus
}
#endif

#endif
