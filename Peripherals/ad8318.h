#include "common.h"

#include "timers.h"

typedef enum {
  AD8318_OK,
  AD8318_OVER_RANGE,
  AD8318_UNDER_RANGE,
} AD8318_Status;
AD8318_Status AD8318_VoltageToDBM(double voltage, double *dbm);