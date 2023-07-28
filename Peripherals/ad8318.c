#include "ad8318.h"

AD8318_Status AD8318_VoltageToDBM(double voltage, double *dbm) {
  // Measured with STM32F103VE ADC. May need to be calibrated for other MCUs.
  if (voltage < 0.6) {
    *dbm = 0;
    return AD8318_OVER_RANGE;
  }
  if (voltage > 1.9) {
    *dbm = -60;
    return AD8318_UNDER_RANGE;
  }
  *dbm = -41.23729408316616 * voltage + 22.800379487797382;
  return AD8318_OK;
}