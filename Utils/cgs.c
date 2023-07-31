#include "cgs.h"
#include <math.h>

double CGS_NumpyRemainder(double x, double y) { return x - y * floor(x / y); }

double CGS_GetBandpassPeak(double f_sample, double f_carrier) {
  return f_sample / 2 -
         fabs(CGS_NumpyRemainder(f_carrier, f_sample) - f_sample / 2);
}
double CGS_ModL2Norm(int n, double *base, double *x, double *y) {
  double sum = 0;
  for (int i = 0; i < n; i++) {
    double dist = MIN(CGS_NumpyRemainder(x[i] - y[i], base[i] / 2),
                      CGS_NumpyRemainder(y[i] - x[i], base[i] / 2));
    sum += dist * dist;
  }
  return sum;
}

double CGS_DetectCarrierFrequencyIteration(int n, double *base,
                                           double *base_coord, double start,
                                           double end, double step) {
  double best_norm = 1e18;
  double best_freq = 0;
  for (double freq = start; freq <= end; freq += step) {
    double candidate_coord[CGS_MAXN] = {0};
    for (int i = 0; i < n; i++) {
      candidate_coord[i] = CGS_GetBandpassPeak(base[i], freq);
    }
    double norm = CGS_ModL2Norm(n, base, base_coord, candidate_coord);
    if (norm < best_norm) {
      best_norm = norm;
      best_freq = freq;
    }
  }
  return best_freq;
}

double CGS_DetectCarrierFrequency(int n, double *base, double *base_coord) {
  double freq_rough = CGS_DetectCarrierFrequencyIteration(n, base, base_coord,
                                                          5e6, 410e6, 0.1e6);
  double freq_fine = CGS_DetectCarrierFrequencyIteration(
      n, base, base_coord, freq_rough - 0.2e6, freq_rough + 0.2e6, 0.01e6);
  return freq_fine;
}
