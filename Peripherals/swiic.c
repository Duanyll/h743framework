#include "swiic.h"

// Initializes the IIC bus
void SWIIC_Init(SWIIC_Config *config) {
#ifdef SWIIC_USE_OPEN_DRAIN
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_OD,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  GPIO_InitStruct.Pin = config->SDA_Pin;
  HAL_GPIO_Init(config->SDA_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = config->SCL_Pin;
  HAL_GPIO_Init(config->SCL_Port, &GPIO_InitStruct);
#else
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  GPIO_InitStruct.Pin = config->SDA_Pin;
  HAL_GPIO_Init(config->SDA_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = config->SCL_Pin;
  HAL_GPIO_Init(config->SCL_Port, &GPIO_InitStruct);
#endif
}

#ifdef SWIIC_USE_OPEN_DRAIN
#define SDA_INPUT()
#define SDA_OUTPUT()
#else
#define SDA_INPUT()                                                            \
  do {                                                                         \
    GPIO_InitTypeDef GPIO_InitStruct = {                                       \
        .Mode = GPIO_MODE_INPUT,                                               \
        .Pull = GPIO_NOPULL,                                                   \
        .Speed = GPIO_SPEED_FREQ_HIGH,                                         \
        .Pin = config->SDA_Pin,                                                \
    };                                                                         \
    HAL_GPIO_Init(config->SDA_Port, &GPIO_InitStruct);                         \
  } while (0)
#define SDA_OUTPUT()                                                           \
  do {                                                                         \
    GPIO_InitTypeDef GPIO_InitStruct = {                                       \
        .Mode = GPIO_MODE_OUTPUT_PP,                                           \
        .Pull = GPIO_NOPULL,                                                   \
        .Speed = GPIO_SPEED_FREQ_HIGH,                                         \
        .Pin = config->SDA_Pin,                                                \
    };                                                                         \
    HAL_GPIO_Init(config->SDA_Port, &GPIO_InitStruct);                         \
  } while (0)
#endif

#define DELAY()                                                                \
  do {                                                                         \
    for (int i = 0; i < config->delay; i++) {                                  \
      __NOP();                                                                 \
    }                                                                          \
  } while (0)
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET
#define WRITE_SDA(x)                                                           \
  HAL_GPIO_WritePin(config->SDA_Port, config->SDA_Pin, (x) ? HIGH : LOW)
#define WRITE_SCL(x) HAL_GPIO_WritePin(config->SCL_Port, config->SCL_Pin, (x))
#define READ_SDA() HAL_GPIO_ReadPin(config->SDA_Port, config->SDA_Pin)

void SWIIC_Start(SWIIC_Config *config) {
  WRITE_SDA(HIGH);
  WRITE_SCL(HIGH);
  DELAY();
  WRITE_SDA(LOW);
  DELAY();
  WRITE_SCL(LOW);
  DELAY();
}

void SWIIC_Stop(SWIIC_Config *config) {
  WRITE_SDA(LOW);
  WRITE_SCL(HIGH);
  DELAY();
  WRITE_SDA(HIGH);
  DELAY();
}

BOOL SWIIC_WaitAck(SWIIC_Config *config) {
  WRITE_SDA(HIGH);
  DELAY();
  SDA_INPUT();
  WRITE_SCL(HIGH);
  DELAY();
  uint8_t ack = READ_SDA();
  int tries = 10;
  while (ack && tries--) {
    DELAY();
    ack = READ_SDA();
  }
  WRITE_SCL(LOW);
  SDA_OUTPUT();
  DELAY();
  return !ack;
}

void SWIIC_WriteAck(SWIIC_Config *config) {
  WRITE_SDA(LOW);
  DELAY();
  WRITE_SCL(HIGH);
  DELAY();
  WRITE_SCL(LOW);
  DELAY();
  WRITE_SDA(HIGH);
  DELAY();
}

void SWIIC_WriteNotAck(SWIIC_Config *config) {
  WRITE_SDA(HIGH);
  DELAY();
  WRITE_SCL(HIGH);
  DELAY();
  WRITE_SCL(LOW);
  DELAY();
}

void SWIIC_WriteByte(SWIIC_Config *config, uint8_t data) {
  WRITE_SCL(LOW);
  for (int i = 7; i >= 0; i--) {
    WRITE_SDA((data >> i) & 1);
    DELAY();
    WRITE_SCL(HIGH);
    DELAY();
    WRITE_SCL(LOW);
  }
  DELAY();
}

uint8_t SWIIC_ReadByte(SWIIC_Config *config) {
  uint8_t data = 0;
  WRITE_SDA(HIGH);
  SDA_INPUT();
  for (int i = 7; i >= 0; i--) {
    WRITE_SCL(HIGH);
    DELAY();
    data |= READ_SDA() << i;
    WRITE_SCL(LOW);
    DELAY();
  }
  SDA_OUTPUT();
  return data;
}

#define CHECK_ACK()                                                            \
  if (!SWIIC_WaitAck(config))                                                  \
    return SWIIC_ERROR;                                                        \
  DELAY();

// Read bytes from the IIC bus. Register address is 8 bits.
SWIIC_State SWIIC_ReadBytes8(SWIIC_Config *config, uint8_t addr, uint8_t reg,
                             uint8_t *data, uint8_t count) {
  SWIIC_Start(config);
  SWIIC_WriteByte(config, addr << 1);
  CHECK_ACK();
  SWIIC_WriteByte(config, reg);
  CHECK_ACK();
  SWIIC_Start(config);
  SWIIC_WriteByte(config, (addr << 1) | 1);
  CHECK_ACK();
  for (int i = 0; i < count; i++) {
    data[i] = SWIIC_ReadByte(config);
    if (i < count - 1) {
      SWIIC_WriteAck(config);
    } else {
      SWIIC_WriteNotAck(config);
    }
  }
  SWIIC_Stop(config);
  return SWIIC_OK;
}
// Read bytes from the IIC bus. Register address is 16 bits.
SWIIC_State SWIIC_ReadBytes16(SWIIC_Config *config, uint8_t addr, uint16_t reg,
                              uint8_t *data, uint8_t count) {
  SWIIC_Start(config);
  SWIIC_WriteByte(config, addr << 1);
  CHECK_ACK();
  SWIIC_WriteByte(config, reg >> 8);
  CHECK_ACK();
  SWIIC_WriteByte(config, reg);
  CHECK_ACK();
  SWIIC_Start(config);
  SWIIC_WriteByte(config, (addr << 1) | 1);
  CHECK_ACK();
  for (int i = 0; i < count; i++) {
    data[i] = SWIIC_ReadByte(config);
    if (i < count - 1) {
      SWIIC_WriteAck(config);
    } else {
      SWIIC_WriteNotAck(config);
    }
  }
  SWIIC_Stop(config);
  return SWIIC_OK;
}
// Write bytes to the IIC bus. Register address is 8 bits.
SWIIC_State SWIIC_WriteBytes8(SWIIC_Config *config, uint8_t addr, uint8_t reg,
                              uint8_t *data, uint8_t count) {
  SWIIC_Start(config);
  SWIIC_WriteByte(config, addr << 1);
  CHECK_ACK();
  SWIIC_WriteByte(config, reg);
  CHECK_ACK();
  for (int i = 0; i < count; i++) {
    SWIIC_WriteByte(config, data[i]);
    CHECK_ACK();
  }
  SWIIC_Stop(config);
  return SWIIC_OK;
}
// Write bytes to the IIC bus. Register address is 16 bits.
SWIIC_State SWIIC_WriteBytes16(SWIIC_Config *config, uint8_t addr, uint16_t reg,
                               uint8_t *data, uint8_t count) {
  SWIIC_Start(config);
  SWIIC_WriteByte(config, addr << 1);
  CHECK_ACK();
  SWIIC_WriteByte(config, reg >> 8);
  CHECK_ACK();
  SWIIC_WriteByte(config, reg);
  CHECK_ACK();
  for (int i = 0; i < count; i++) {
    SWIIC_WriteByte(config, data[i]);
    CHECK_ACK();
  }
  SWIIC_Stop(config);
  return SWIIC_OK;
}
// Check if a device is present on the IIC bus.
SWIIC_State SWIIC_CheckDevice(SWIIC_Config *config, uint8_t addr) {
  SWIIC_Start(config);
  SWIIC_WriteByte(config, addr << 1);
  if (!SWIIC_WaitAck(config)) {
    SWIIC_Stop(config);
    return SWIIC_ERROR;
  }
  SWIIC_Stop(config);
  return SWIIC_OK;
}