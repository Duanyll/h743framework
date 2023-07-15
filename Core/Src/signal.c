#include "arm_math.h"

#include "signal.h"

#ifdef SIGNAL_Q15_ENABLE 

arm_cfft_radix4_instance_q15 SIGNAL_fftInstanceQ15;
int16_t SIGNAL_fftBufferQ15[SIGNAL_MAX_POINTS * 2];
int16_t SIGNAL_magBufferQ15[SIGNAL_MAX_POINTS];

void SIGNAL_TimeQ15ToSpectrumQ15(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumQ15 *freqData) {
  int idx = timeData->timeDataOffset;
  for (int i = 0; i < timeData->points; i++) {
    SIGNAL_fftBufferQ15[i * 2] = timeData->timeData[idx];
    SIGNAL_fftBufferQ15[i * 2 + 1] = 0;
    idx += timeData->timeDataStride;
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

arm_cfft_radix4_instance_f32 SIGNAL_fftInstanceF32;
float32_t SIGNAL_fftBufferF32[SIGNAL_MAX_POINTS * 2];
float32_t SIGNAL_magBufferF32[SIGNAL_MAX_POINTS];

void SIGNAL_TimeQ15ToSpectrumF32(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumF32 *freqData) {
  int idx = timeData->timeDataOffset;
  for (int i = 0; i < timeData->points; i++) {
    SIGNAL_fftBufferF32[i * 2] = timeData->timeData[idx];
    SIGNAL_fftBufferF32[i * 2 + 1] = 0;
    idx += timeData->timeDataStride;
  }
  SIGNAL_fftInstanceF32.fftLen = timeData->points;
  arm_cfft_radix4_init_f32(&SIGNAL_fftInstanceF32, timeData->points, 0, 1);
  arm_cfft_radix4_f32(&SIGNAL_fftInstanceF32, SIGNAL_fftBufferF32);
  arm_cmplx_mag_f32(SIGNAL_fftBufferF32, SIGNAL_magBufferF32, timeData->points);
  freqData->points = timeData->points / 2;
  freqData->sampleRate = timeData->sampleRate;
  freqData->ampData = SIGNAL_magBufferF32;
  freqData->cfftData = SIGNAL_fftBufferF32;
  freqData->dc = SIGNAL_magBufferF32[0];
  freqData->peakAmp = 0;
  freqData->peakFreq = 0;
  for (int i = 1; i < freqData->points; i++) {
    if (SIGNAL_magBufferF32[i] > freqData->peakAmp) {
      freqData->peakAmp = SIGNAL_magBufferF32[i];
      freqData->peakFreq = i * timeData->sampleRate / timeData->points;
    }
  }
}

#endif