/*---------------------------------------
- WeAct Studio Official Link
- taobao: weactstudio.taobao.com
- aliexpress: weactstudio.aliexpress.com
- github: github.com/WeActTC
- gitee: gitee.com/WeAct-TC
- blog: www.weact-tc.cn
---------------------------------------*/

#include "zjy122250/epaper.h"
#include "zjy122250/epdfont.h"

#include "common.h"
#include "timers.h"

static EPD_Pins* pins;
#define WRITE(pin, state) HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, state)

EPD_PAINT EPD_Paint;

static uint8_t _hibernating = 1;

static const unsigned char ut_partial[] = {
    0x0,  0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x80,
    0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x40, 0x40,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x80, 0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0A, 0x0, 0x0, 0x0,  0x0,
    0x0,  0x2,  0x1,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x1, 0x0, 0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0,  0x0,
    0x0,  0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0,  0x0,
};

void EPD_Delay() {
  for (int i = 0; i < 20; i++) {
    __NOP();
  }
}

uint8_t EPD_is_busy() {
  return HAL_GPIO_ReadPin(pins->BUSY_Port, pins->BUSY_Pin) == GPIO_PIN_SET;
}

void EPD_io_init(EPD_Pins *p) {
  pins = p;

  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
#define INIT(pin)                                                              \
  s.Pin = pins->pin##_Pin;                                                     \
  HAL_GPIO_Init(pins->pin##_Port, &s)
  INIT(RES);
  INIT(DC);
  INIT(CS);
  INIT(SCL);
  INIT(SDA);
  s.Mode = GPIO_MODE_INPUT;
  INIT(BUSY);
#undef INIT

  WRITE(RES, LOW);
  WRITE(DC, HIGH);
  WRITE(CS, HIGH);
  WRITE(SCL, LOW);
  WRITE(SDA, LOW);
}

void EPD_WriteByte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    WRITE(SCL, LOW);
    WRITE(SDA, (data >> i) & 0x01);
    EPD_Delay();
    WRITE(SCL, HIGH);
    EPD_Delay();
  }
  WRITE(SCL, LOW);
}

void EPD_write_reg(uint8_t reg) {
  WRITE(DC, LOW);
  WRITE(CS, LOW);

  EPD_WriteByte(reg);

  WRITE(CS, HIGH);
  WRITE(DC, HIGH);
}

void EPD_write_data(uint8_t data) {
  WRITE(CS, LOW);

  EPD_WriteByte(data);

  WRITE(CS, HIGH);
}

uint8_t EPD_wait_busy() {
  uint32_t timeout = 0;
  while (EPD_is_busy()) {
    timeout++;
    if (timeout > 40000) {
      return 1;
    }
    HAL_Delay(1);
  }
  return 0;
}

void EPD_reset(void) {
  WRITE(RES, LOW);
  HAL_Delay(50);
  WRITE(RES, HIGH);
  HAL_Delay(50);
  _hibernating = 0;
}

uint8_t EPD_init(void) {
  if (_hibernating)
    EPD_reset();

  if (EPD_wait_busy())
    return 1;

  EPD_write_reg(0x12); // SWRESET

  if (EPD_wait_busy())
    return 1;

  EPD_write_reg(0x01); // Driver output control
  EPD_write_data(0x27);
  EPD_write_data(0x01);
  EPD_write_data(0x01);

  EPD_write_reg(0x11); // data entry mode
  EPD_write_data(0x01);

  EPD_write_reg(0x44); // set Ram-X address start/end position
  EPD_write_data(0x00);
  EPD_write_data(0x0F); // 0x0F-->(15+1)*8=128

  EPD_write_reg(0x45);  // set Ram-Y address start/end position
  EPD_write_data(0x27); // 0x127-->(295+1)=296
  EPD_write_data(0x01);
  EPD_write_data(0x00);
  EPD_write_data(0x00);

  EPD_write_reg(0x3C); // BorderWavefrom
  EPD_write_data(0x05);

  EPD_write_reg(0x21); //  Display update control
  EPD_write_data(0x00);
  EPD_write_data(0x80);

  EPD_write_reg(0x18); // Read built-in temperature sensor
  EPD_write_data(0x80);

  EPD_write_reg(0x4E); // set RAM x address count to 0;
  EPD_write_data(0x00);
  EPD_write_reg(0x4F); // set RAM y address count to 0x127;
  EPD_write_data(0x27);
  EPD_write_data(0x01);

  if (EPD_wait_busy())
    return 1;

  return 0;
}

uint8_t EPD_init_partial(void) {
  if (EPD_init())
    return 1;

  EPD_write_reg(0x32);
  WRITE(CS, LOW);
  for (int j = 0; j < sizeof(ut_partial); j++) {
    EPD_WriteByte(ut_partial[j]);
  }
  WRITE(CS, HIGH);

  return 0;
}

void EPD_enter_deepsleepmode(uint8_t mode) {
  EPD_write_reg(0x10);
  EPD_write_data(mode);
  _hibernating = 1;
}

void EPD_init_internalTempSensor(void) {
  EPD_write_reg(0x18);
  EPD_write_data(0x80);

  EPD_write_reg(0x1A);
  EPD_write_data(0x7F);
  EPD_write_data(0xF0);
}

void EPD_update(void) {
  EPD_write_reg(0x22); // Display Update Control
  EPD_write_data(0xF7);
  EPD_write_reg(0x20); // Activate Display Update Sequence

  EPD_wait_busy();
}

void EPD_update_partial(void) {
  EPD_write_reg(0x22); // Display Update Control
  EPD_write_data(0xCC);
  EPD_write_reg(0x20); // Activate Display Update Sequence

  EPD_wait_busy();
}

void EPD_setpos(uint16_t x, uint16_t y) {
  uint8_t _x;
  uint16_t _y;

  _x = x / 8;

  _y = 295 - y;

  EPD_write_reg(0x4E); // set RAM x address count to 0;
  EPD_write_data(_x);
  EPD_write_reg(0x4F); // set RAM y address count to 0x127;
  EPD_write_data(_y & 0xff);
  EPD_write_data(_y >> 8 & 0x01);
}

void EPD_writedata(uint8_t *Image1, uint32_t length) {
  WRITE(CS, LOW);
  for (uint32_t j = 0; j < length; j++) {
    EPD_WriteByte(Image1[j]);
  }
  WRITE(CS, HIGH);
}

void EPD_display(uint8_t *Image1, uint8_t *Image2) {
  uint32_t Width, Height, i, j;
  uint32_t k = 0;
  Width = EPD_H;
  Height = EPD_W_BUFF_SIZE;

  EPD_setpos(0, 0);

  EPD_write_reg(0x24);
  EPD_writedata(Image1, Width * Height);

  EPD_setpos(0, 0);

  EPD_write_reg(0x26);
  k = 0;
  WRITE(CS, LOW);
  for (j = 0; j < Height; j++) {
    for (i = 0; i < Width; i++) {
      EPD_WriteByte(~Image2[k]);
      k++;
    }
  }
  WRITE(CS, HIGH);

  EPD_update();
}

void EPD_displayBW(uint8_t *Image) {
  uint32_t Width, Height;

  Width = EPD_H;
  Height = EPD_W_BUFF_SIZE;

  EPD_setpos(0, 0);
  EPD_write_reg(0x26);
  EPD_writedata(Image, Width * Height);

  EPD_setpos(0, 0);
  EPD_write_reg(0x24);
  EPD_writedata(Image, Width * Height);

  EPD_update();
}

void EPD_displayBW_partial(uint8_t *Image) {
  uint32_t Width, Height;

  Width = EPD_H;
  Height = EPD_W_BUFF_SIZE;

  EPD_setpos(0, 0);
  EPD_write_reg(0x24);
  EPD_writedata(Image, Width * Height);

  EPD_update_partial();

  EPD_setpos(0, 0);
  EPD_write_reg(0x26);
  EPD_writedata(Image, Width * Height);
}

void EPD_displayRED(uint8_t *Image) {
  uint32_t Width, Height;

  Width = EPD_H;
  Height = EPD_W_BUFF_SIZE;

  EPD_setpos(0, 0);

  EPD_write_reg(0x26);
  EPD_writedata(Image, Width * Height);

  EPD_update();
}

void EPD_paint_newimage(uint8_t *image, uint16_t Width, uint16_t Height,
                        uint16_t Rotate, uint16_t Color) {
  EPD_Paint.Image = 0x00;
  EPD_Paint.Image = image;

  EPD_Paint.WidthMemory = Width;
  EPD_Paint.HeightMemory = Height;
  EPD_Paint.Color = Color;
  EPD_Paint.WidthByte = (Width % 8 == 0) ? (Width / 8) : (Width / 8 + 1);
  EPD_Paint.HeightByte = Height;
  EPD_Paint.Rotate = Rotate;
  if (Rotate == EPD_ROTATE_0 || Rotate == EPD_ROTATE_180) {

    EPD_Paint.Width = Height;
    EPD_Paint.Height = Width;
  } else {
    EPD_Paint.Width = Width;
    EPD_Paint.Height = Height;
  }
}

void EPD_paint_setpixel(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color) {
  uint16_t X, Y;
  uint32_t Addr;
  uint8_t Rdata;
  switch (EPD_Paint.Rotate) {
  case 0:

    X = EPD_Paint.WidthMemory - Ypoint - 1;
    Y = Xpoint;
    break;
  case 90:
    X = EPD_Paint.WidthMemory - Xpoint - 1;
    Y = EPD_Paint.HeightMemory - Ypoint - 1;
    break;
  case 180:
    X = Ypoint;
    Y = EPD_Paint.HeightMemory - Xpoint - 1;
    break;
  case 270:
    X = Xpoint;
    Y = Ypoint;
    break;
  default:
    return;
  }
  Addr = X / 8 + Y * EPD_Paint.WidthByte;
  Rdata = EPD_Paint.Image[Addr];
  if (Color == EPD_COLOR_BLACK) {
    EPD_Paint.Image[Addr] = Rdata & ~(0x80 >> (X % 8));
  } else
    EPD_Paint.Image[Addr] = Rdata | (0x80 >> (X % 8));
}

void EPD_paint_clear(uint16_t color) {
  uint16_t X, Y;
  uint32_t Addr;

  for (Y = 0; Y < EPD_Paint.HeightByte; Y++) {
    for (X = 0; X < EPD_Paint.WidthByte; X++) { // 8 pixel =  1 byte
      Addr = X + Y * EPD_Paint.WidthByte;
      EPD_Paint.Image[Addr] = color;
    }
  }
}

void EPD_paint_selectimage(uint8_t *image) { EPD_Paint.Image = image; }

void EPD_paint_drawPoint(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color) {
  EPD_paint_setpixel(Xpoint - 1, Ypoint - 1, Color);
}

void EPD_paint_drawLine(uint16_t Xstart, uint16_t Ystart, uint16_t Xend,
                        uint16_t Yend, uint16_t Color) {
  uint16_t Xpoint, Ypoint;
  int32_t dx, dy;
  int32_t XAddway, YAddway;
  int32_t Esp;
  char Dotted_Len;
  Xpoint = Xstart;
  Ypoint = Ystart;
  dx = (int)Xend - (int)Xstart >= 0 ? Xend - Xstart : Xstart - Xend;
  dy = (int)Yend - (int)Ystart <= 0 ? Yend - Ystart : Ystart - Yend;

  XAddway = Xstart < Xend ? 1 : -1;
  YAddway = Ystart < Yend ? 1 : -1;

  Esp = dx + dy;
  Dotted_Len = 0;

  for (;;) {
    Dotted_Len++;
    EPD_paint_drawPoint(Xpoint, Ypoint, Color);
    if (2 * Esp >= dy) {
      if (Xpoint == Xend)
        break;
      Esp += dy;
      Xpoint += XAddway;
    }
    if (2 * Esp <= dx) {
      if (Ypoint == Yend)
        break;
      Esp += dx;
      Ypoint += YAddway;
    }
  }
}

void EPD_paint_drawRectangle(uint16_t Xstart, uint16_t Ystart, uint16_t Xend,
                             uint16_t Yend, uint16_t Color, uint8_t mode) {
  uint16_t i;
  if (mode) {
    for (i = Ystart; i < Yend; i++) {
      EPD_paint_drawLine(Xstart, i, Xend, i, Color);
    }
  } else {
    EPD_paint_drawLine(Xstart, Ystart, Xend, Ystart, Color);
    EPD_paint_drawLine(Xstart, Ystart, Xstart, Yend, Color);
    EPD_paint_drawLine(Xend, Yend, Xend, Ystart, Color);
    EPD_paint_drawLine(Xend, Yend, Xstart, Yend, Color);
  }
}

void EPD_paint_drawCircle(uint16_t X_Center, uint16_t Y_Center, uint16_t Radius,
                          uint16_t Color, uint8_t mode) {
  uint16_t Esp, sCountY;
  uint16_t XCurrent, YCurrent;
  XCurrent = 0;
  YCurrent = Radius;
  Esp = 3 - (Radius << 1);
  if (mode) {
    while (XCurrent <= YCurrent) { // Realistic circles
      for (sCountY = XCurrent; sCountY <= YCurrent; sCountY++) {
        EPD_paint_drawPoint(X_Center + XCurrent, Y_Center + sCountY,
                            Color); // 1
        EPD_paint_drawPoint(X_Center - XCurrent, Y_Center + sCountY,
                            Color); // 2
        EPD_paint_drawPoint(X_Center - sCountY, Y_Center + XCurrent,
                            Color); // 3
        EPD_paint_drawPoint(X_Center - sCountY, Y_Center - XCurrent,
                            Color); // 4
        EPD_paint_drawPoint(X_Center - XCurrent, Y_Center - sCountY,
                            Color); // 5
        EPD_paint_drawPoint(X_Center + XCurrent, Y_Center - sCountY,
                            Color); // 6
        EPD_paint_drawPoint(X_Center + sCountY, Y_Center - XCurrent,
                            Color); // 7
        EPD_paint_drawPoint(X_Center + sCountY, Y_Center + XCurrent, Color);
      }
      if ((int)Esp < 0)
        Esp += 4 * XCurrent + 6;
      else {
        Esp += 10 + 4 * (XCurrent - YCurrent);
        YCurrent--;
      }
      XCurrent++;
    }
  } else { // Draw a hollow circle
    while (XCurrent <= YCurrent) {
      EPD_paint_drawPoint(X_Center + XCurrent, Y_Center + YCurrent, Color); // 1
      EPD_paint_drawPoint(X_Center - XCurrent, Y_Center + YCurrent, Color); // 2
      EPD_paint_drawPoint(X_Center - YCurrent, Y_Center + XCurrent, Color); // 3
      EPD_paint_drawPoint(X_Center - YCurrent, Y_Center - XCurrent, Color); // 4
      EPD_paint_drawPoint(X_Center - XCurrent, Y_Center - YCurrent, Color); // 5
      EPD_paint_drawPoint(X_Center + XCurrent, Y_Center - YCurrent, Color); // 6
      EPD_paint_drawPoint(X_Center + YCurrent, Y_Center - XCurrent, Color); // 7
      EPD_paint_drawPoint(X_Center + YCurrent, Y_Center + XCurrent, Color); // 0
      if ((int)Esp < 0)
        Esp += 4 * XCurrent + 6;
      else {
        Esp += 10 + 4 * (XCurrent - YCurrent);
        YCurrent--;
      }
      XCurrent++;
    }
  }
}

void EPD_paint_showChar(uint16_t x, uint16_t y, uint16_t chr, uint16_t size1,
                        uint16_t color) {
  uint16_t i, m, temp, size2, chr1;
  uint16_t x0, y0;
  x += 1, y += 1, x0 = x, y0 = y;
  if (x - size1 > EPD_H)
    return;
  if (size1 == 8)
    size2 = 6;
  else
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
  chr1 = chr - ' ';
  for (i = 0; i < size2; i++) {
    if (size1 == 8) {
      temp = asc2_0806[chr1][i];
    } // 0806
    else if (size1 == 12) {
      temp = asc2_1206[chr1][i];
    } // 1206
    else if (size1 == 16) {
      temp = asc2_1608[chr1][i];
    } // 1608
    else if (size1 == 24) {
      temp = asc2_2412[chr1][i];
    } // 2412
    else
      return;
    for (m = 0; m < 8; m++) {
      if (temp & 0x01)
        EPD_paint_drawPoint(x, y, color);
      else
        EPD_paint_drawPoint(x, y, !color);
      temp >>= 1;
      y++;
    }
    x++;
    if ((size1 != 8) && ((x - x0) == size1 / 2)) {
      x = x0;
      y0 = y0 + 8;
    }
    y = y0;
  }
}

void EPD_paint_showString(uint16_t x, uint16_t y, char *chr, uint16_t size1,
                          uint16_t color) {
  while (*chr != '\0') {
    EPD_paint_showChar(x, y, *chr, size1, color);
    chr++;
    if (size1 == 8) {
      x += 6;
    } else {
      x += size1 / 2;
    }
  }
}

// m^n
static uint32_t _Pow(uint16_t m, uint16_t n) {
  uint32_t result = 1;
  while (n--) {
    result *= m;
  }
  return result;
}

void EPD_paint_showNum(uint16_t x, uint16_t y, uint32_t num, uint16_t len,
                       uint16_t size1, uint16_t color) {
  uint8_t t, temp, m = 0;
  if (size1 == 8)
    m = 2;
  for (t = 0; t < len; t++) {
    temp = (num / _Pow(10, len - t - 1)) % 10;
    if (temp == 0) {
      EPD_paint_showChar(x + (size1 / 2 + m) * t, y, '0', size1, color);
    } else {
      EPD_paint_showChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, color);
    }
  }
}

void EPD_paint_showChinese(uint16_t x, uint16_t y, uint16_t num, uint16_t size1,
                           uint16_t color) {
  uint16_t m, temp;
  uint16_t x0, y0;
  uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;
  x += 1, y += 1, x0 = x, y0 = y;
  for (i = 0; i < size3; i++) {
    if (size1 == 16) {
      temp = Hzk1[num][i];
    } // 16*16
    else if (size1 == 24) {
      temp = Hzk2[num][i];
    } // 24*24
    else if (size1 == 32) {
      temp = Hzk3[num][i];
    } // 32*32
    else if (size1 == 64) {
      temp = Hzk4[num][i];
    } // 64*64
    else
      return;
    for (m = 0; m < 8; m++) {
      if (temp & 0x01)
        EPD_paint_drawPoint(x, y, color);
      else
        EPD_paint_drawPoint(x, y, !color);
      temp >>= 1;
      y++;
    }
    x++;
    if ((x - x0) == size1) {
      x = x0;
      y0 = y0 + 8;
    }
    y = y0;
  }
}

void EPD_paint_showPicture(uint16_t x, uint16_t y, uint16_t sizex,
                           uint16_t sizey, const uint8_t BMP[],
                           uint16_t Color) {
  uint16_t j = 0;
  uint16_t i, n = 0, temp = 0, m = 0;
  uint16_t x0 = 0, y0 = 0;
  x += 1, y += 1, x0 = x, y0 = y;
  sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
  for (n = 0; n < sizey; n++) {
    for (i = 0; i < sizex; i++) {
      temp = BMP[j];
      j++;
      for (m = 0; m < 8; m++) {
        if (temp & 0x01)
          EPD_paint_drawPoint(x, y, !Color);
        else
          EPD_paint_drawPoint(x, y, Color);
        temp >>= 1;
        y++;
      }
      x++;
      if ((x - x0) == sizex) {
        x = x0;
        y0 = y0 + 8;
      }
      y = y0;
    }
  }
}
