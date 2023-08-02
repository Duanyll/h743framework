#include "signal.h"

#if defined(ARM_MATH_CM3) || defined(ARM_MATH_CM4) || defined(ARM_MATH_CM7) || \
    defined(ARM_MATH_CM0) || defined(ARM_MATH_CM0PLUS)

#include "arm_math.h"

void SIGNAL_FFTImplF32(SIGNAL_SpectrumF32 *freqData, int points,
                       double sampleRate, SIGNAL_FFTBufferF32 *buffer,
                       BOOL stripDc) {
  if (points == 64 || points == 256 || points == 1024 || points == 4096) {
    buffer->rad4Instance.fftLen = points;
    arm_cfft_radix4_init_f32(&buffer->rad4Instance, points, 0, 1);
    arm_cfft_radix4_f32(&buffer->rad4Instance, buffer->fftBuffer);
  } else {
    buffer->rad2Instance.fftLen = points;
    arm_cfft_radix2_init_f32(&buffer->rad2Instance, points, 0, 1);
    arm_cfft_radix2_f32(&buffer->rad2Instance, buffer->fftBuffer);
  }
  arm_cmplx_mag_f32(buffer->fftBuffer, buffer->magBuffer, points);
  freqData->points = points / 2;
  freqData->sampleRate = sampleRate;
  freqData->ampData = buffer->magBuffer;
  freqData->cfftData = buffer->fftBuffer;
  freqData->peakAmp = 0;
  freqData->peakFreq = 0;
  if (stripDc) {
    BOOL dcLeakFlag = TRUE;
    for (int i = 2; i < freqData->points; i++) {
      if (dcLeakFlag && buffer->magBuffer[i] < buffer->magBuffer[i - 1]) {
        continue;
      } else {
        dcLeakFlag = FALSE;
      }
      if (buffer->magBuffer[i] > freqData->peakAmp) {
        freqData->peakAmp = buffer->magBuffer[i];
        freqData->peakFreq = i * sampleRate / points;
      }
    }
  } else {
    for (int i = 0; i < freqData->points; i++) {
      if (buffer->magBuffer[i] > freqData->peakAmp) {
        freqData->peakAmp = buffer->magBuffer[i];
        freqData->peakFreq = i * sampleRate / points;
      }
    }
  }
}

void SIGNAL_TimeQ15ToSpectrumF32(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumF32 *freqData,
                                 SIGNAL_FFTBufferF32 *buffer) {
  int idx = timeData->offset;
  int32_t mean = 0;
  if (timeData->stripDc) {
    for (int i = 0; i < timeData->points; i++) {
      mean += timeData->timeData[idx];
      idx += timeData->stride;
    }
    mean /= timeData->points;
  }
  idx = timeData->offset;
  for (int i = 0; i < timeData->points; i++) {
    buffer->fftBuffer[i * 2] =
        (timeData->timeData[idx] - mean) / 32768.0f * timeData->range;
    buffer->fftBuffer[i * 2 + 1] = 0;
    idx += timeData->stride;
  }
  freqData->dc = mean / 32768.0f * timeData->range;
  SIGNAL_FFTImplF32(freqData, timeData->points, timeData->sampleRate, buffer,
                    timeData->stripDc);
}

void SIGNAL_TimeF32ToSpectrumF32(SIGNAL_TimeDataF32 *timeData,
                                 SIGNAL_SpectrumF32 *freqData,
                                 SIGNAL_FFTBufferF32 *buffer) {
  int idx = timeData->offset;
  float32_t mean = 0;
  if (timeData->stripDc) {
    for (int i = 0; i < timeData->points; i++) {
      mean += timeData->timeData[idx];
      idx += timeData->stride;
    }
    mean /= timeData->points;
  }
  idx = timeData->offset;
  for (int i = 0; i < timeData->points; i++) {
    buffer->fftBuffer[i * 2] = timeData->timeData[idx] - mean;
    buffer->fftBuffer[i * 2 + 1] = 0;
    idx += timeData->stride;
  }
  freqData->dc = mean;
  SIGNAL_FFTImplF32(freqData, timeData->points, timeData->sampleRate, buffer,
                    timeData->stripDc);
}

#endif

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