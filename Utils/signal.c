#include "arm_math.h"

#include "signal.h"

#ifdef SIGNAL_Q15_ENABLE

arm_cfft_radix4_instance_q15 SIGNAL_fftInstanceQ15;
int16_t SIGNAL_fftBufferQ15[SIGNAL_MAX_POINTS * 2];
int16_t SIGNAL_magBufferQ15[SIGNAL_MAX_POINTS];

void SIGNAL_TimeQ15ToSpectrumQ15(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumQ15 *freqData) {
  int idx = timeData->offset;
  for (int i = 0; i < timeData->points; i++) {
    SIGNAL_fftBufferQ15[i * 2] = timeData->timeData[idx];
    SIGNAL_fftBufferQ15[i * 2 + 1] = 0;
    idx += timeData->stride;
  }
  SIGNAL_fftInstanceQ15.fftLen = timeData->points;
  arm_cfft_radix4_init_q15(&SIGNAL_fftInstanceQ15, timeData->points, 0, 1);
  arm_cfft_radix4_q15(&SIGNAL_fftInstanceQ15, SIGNAL_fftBufferQ15);
  arm_cmplx_mag_q15(SIGNAL_fftBufferQ15, SIGNAL_magBufferQ15, timeData->points);
  freqData->points = timeData->points / 2;
  freqData->sampleRate = timeData->sampleRate;
  freqData->ampData = SIGNAL_magBufferQ15;
  freqData->cfftData = SIGNAL_fftBufferQ15;
  freqData->dc = SIGNAL_magBufferQ15[0];
  freqData->peakAmp = 0;
  freqData->peakFreq = 0;
  for (int i = 1; i < freqData->points; i++) {
    if (SIGNAL_magBufferQ15[i] > freqData->peakAmp) {
      freqData->peakAmp = SIGNAL_magBufferQ15[i];
      freqData->peakFreq = i * timeData->sampleRate / timeData->points;
    }
  }
}

#endif

#ifdef SIGNAL_F32_ENABLE

arm_cfft_radix2_instance_f32 SIGNAL_fftInstance2F32;
arm_cfft_radix4_instance_f32 SIGNAL_fftInstance4F32;
float32_t SIGNAL_fftBufferF32[SIGNAL_MAX_POINTS * 2];
float32_t SIGNAL_magBufferF32[SIGNAL_MAX_POINTS];

void SIGNAL_FFTImplF32(SIGNAL_SpectrumF32 *freqData, int points,
                       double sampleRate) {
  if (points == 64 || points == 256 || points == 1024 || points == 4096) {
    SIGNAL_fftInstance4F32.fftLen = points;
    arm_cfft_radix4_init_f32(&SIGNAL_fftInstance4F32, points, 0, 1);
    arm_cfft_radix4_f32(&SIGNAL_fftInstance4F32, SIGNAL_fftBufferF32);
  } else {
    SIGNAL_fftInstance2F32.fftLen = points;
    arm_cfft_radix2_init_f32(&SIGNAL_fftInstance2F32, points, 0, 1);
    arm_cfft_radix2_f32(&SIGNAL_fftInstance2F32, SIGNAL_fftBufferF32);
  }
  arm_cmplx_mag_f32(SIGNAL_fftBufferF32, SIGNAL_magBufferF32, points);
  freqData->points = points / 2;
  freqData->sampleRate = sampleRate;
  freqData->ampData = SIGNAL_magBufferF32;
  freqData->cfftData = SIGNAL_fftBufferF32;
  freqData->peakAmp = 0;
  freqData->peakFreq = 0;
  BOOL dcLeakFlag = TRUE;
  for (int i = 2; i < freqData->points; i++) {
    if (dcLeakFlag && SIGNAL_magBufferF32[i] < SIGNAL_magBufferF32[i - 1]) {
      continue;
    } else {
      dcLeakFlag = FALSE;
    }
    if (SIGNAL_magBufferF32[i] > freqData->peakAmp) {
      freqData->peakAmp = SIGNAL_magBufferF32[i];
      freqData->peakFreq = i * sampleRate / points;
    }
  }
}

void SIGNAL_TimeQ15ToSpectrumF32(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumF32 *freqData) {
  int idx = timeData->offset;
  int32_t mean = 0;
  for (int i = 0; i < timeData->points; i++) {
    mean += timeData->timeData[idx];
    idx += timeData->stride;
  }
  mean /= timeData->points;
  idx = timeData->offset;
  for (int i = 0; i < timeData->points; i++) {
    SIGNAL_fftBufferF32[i * 2] =
        (timeData->timeData[idx] - mean) / 32768.0f * timeData->range;
    SIGNAL_fftBufferF32[i * 2 + 1] = 0;
    idx += timeData->stride;
  }
  freqData->dc = mean / 32768.0f * timeData->range;
  SIGNAL_FFTImplF32(freqData, timeData->points, timeData->sampleRate);
}

void SIGNAL_TimeF32ToSpectrumF32(SIGNAL_TimeDataF32 *timeData,
                                 SIGNAL_SpectrumF32 *freqData) {
  int idx = timeData->offset;
  float32_t mean = 0;
  for (int i = 0; i < timeData->points; i++) {
    mean += timeData->timeData[idx];
    idx += timeData->stride;
  }
  mean /= timeData->points;
  idx = timeData->offset;
  for (int i = 0; i < timeData->points; i++) {
    SIGNAL_fftBufferF32[i * 2] = timeData->timeData[idx] - mean;
    SIGNAL_fftBufferF32[i * 2 + 1] = 0;
    idx += timeData->stride;
  }
  freqData->dc = mean;
  SIGNAL_FFTImplF32(freqData, timeData->points, timeData->sampleRate);
}

void SIGNAL_FindPeaksF32(SIGNAL_SpectrumF32 *freqData, SIGNAL_PeaksF32 *peaks,
                         float height, int distance) {
  peaks->count = 0;
  int lastPeak = -distance;
  float *amp = freqData->ampData;
  for (int i = 1; i < freqData->points - 1; i++) {
    if (amp[i] > height && amp[i] > amp[i - 1] && amp[i] > amp[i + 1] &&
        i - lastPeak > distance) {
      if (i - lastPeak > distance && peaks->count < SIGNAL_MAX_PEAKS) {
        peaks->peaks[peaks->count].amp = amp[i];
        peaks->peaks[peaks->count].freq =
            i * freqData->sampleRate / freqData->points;
        peaks->count++;
        lastPeak = i;
      } else if (amp[i] > peaks->peaks[peaks->count - 1].amp) {
        peaks->peaks[peaks->count - 1].amp = amp[i];
        peaks->peaks[peaks->count - 1].freq =
            i * freqData->sampleRate / freqData->points;
        lastPeak = i;
      }
    }
  }
}

double SIGNAL_GetCorrelationF32(const float *x, const float *y, int n) {
  double sumX = 0;
  double sumY = 0;
  double sumXY = 0;
  double sumX2 = 0;
  double sumY2 = 0;
  for (int i = 0; i < n; i++) {
    sumX += x[i];
    sumY += y[i];
    sumXY += x[i] * y[i];
    sumX2 += x[i] * x[i];
    sumY2 += y[i] * y[i];
  }
  double numerator = n * sumXY - sumX * sumY;
  double denominator =
      sqrt(n * sumX2 - sumX * sumX) * sqrt(n * sumY2 - sumY * sumY);
  return numerator / denominator;
}

#define M_PI 3.14159265358979323846

void SIGNAL_UnwrapPhaseF32(float *phase, int n) {
  float last = phase[0];
  for (int i = 1; i < n; i++) {
    float diff = phase[i] - last;
    if (diff > M_PI) {
      phase[i] -= 2 * M_PI;
    } else if (diff < -M_PI) {
      phase[i] += 2 * M_PI;
    }
    last = phase[i];
  }
}

double SIGNAL_SimpleSNR(SIGNAL_SpectrumF32 *freqData) {
  double mean = 0;
  for (int i = 1; i < freqData->points; i++) {
    mean += freqData->ampData[i];
  }
  mean /= freqData->points - 1;
  return freqData->peakAmp / mean;
}

#endif