#pragma once

#include "main.h"

#define SIGNAL_MAX_POINTS 1024

typedef struct SIGNAL_TimeDataQ15 {
  int points;         // number of points in the signal
  int timeDataOffset; // offset of the time data in the buffer. used to handle
                      // multiple signals in the same buffer.
  int timeDataStride; // stride of the time data in the buffer. used to handle
                      // multiple signals in the same buffer.
  double sampleRate;  // sample rate of the signal
  int16_t *timeData;  // pointer to the time data. by setting timeDataOffset and
                      // timeDataStride, multiple signals can be stored in the
                      // same buffer.
} SIGNAL_TimeDataQ15;

typedef struct SIGNAL_SpectrumQ15 {
  int points;        // number of points in the spectrum
  double sampleRate; // sample rate of the signal
  int16_t *ampData;  // pointer to the amplitude data
  int16_t *cfftData; // pointer to the complex FFT data
  double dc;         // dc component
  double peakFreq;   // frequency of the peak
  double peakAmp;    // amplitude of the peak
} SIGNAL_SpectrumQ15;

typedef struct SIGNAL_SpectrumF32 {
  int points;        // number of points in the spectrum
  double sampleRate; // sample rate of the signal
  float *ampData;    // pointer to the amplitude data
  float *cfftData;   // pointer to the complex FFT data
  double dc;         // dc component
  double peakFreq;   // frequency of the peak
  double peakAmp;    // amplitude of the peak
} SIGNAL_SpectrumF32;

#ifdef SIGNAL_Q15_ENABLE
void SIGNAL_TimeQ15ToSpectrumQ15(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumQ15 *freqData);
#endif
#ifdef SIGNAL_F32_ENABLE
void SIGNAL_TimeQ15ToSpectrumF32(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumF32 *freqData);
#endif
