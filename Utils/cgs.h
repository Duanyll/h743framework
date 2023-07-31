#include "common.h"

#define CGS_MAXN 16

double CGS_GetBandpassPeak(double f_sample, double f_carrier);
double CGS_ModL2Norm(int n, double *base, double* x, double* y);
double CGS_DetectCarrierFrequency(int n, double* base, double* base_coord);