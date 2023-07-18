#pragma once

#include "main.h"

#define SIGNAL_MAX_POINTS 1024
#define SIGNAL_MAX_PEAKS 32

typedef struct SIGNAL_TimeDataQ15 {
  int points;         // number of points in the signal
  int offset; // offset of the time data in the buffer. used to handle
                      // multiple signals in the same buffer.
  int stride; // stride of the time data in the buffer. used to handle
                      // multiple signals in the same buffer.
  double sampleRate;  // sample rate of the signal
  int16_t *timeData;  // pointer to the time data. by setting offset and
                      // stride, multiple signals can be stored in the
                      // same buffer.
  float range;        // range of the signal +/- range
} SIGNAL_TimeDataQ15;

typedef struct SIGNAL_TimeDataF32 {
  int points;
  int offset;
  int stride;
  double sampleRate;
  float *timeData;
} SIGNAL_TimeDataF32;

#ifdef SIGNAL_Q15_ENABLE

    typedef struct SIGNAL_SpectrumQ15 {
  int points;        // number of points in the spectrum
  double sampleRate; // sample rate of the signal
  int16_t *ampData;  // pointer to the amplitude data
  int16_t *cfftData; // pointer to the complex FFT data
  int16_t dc;        // dc component
  double peakFreq;   // frequency of the peak
  int16_t peakAmp;   // amplitude of the peak
} SIGNAL_SpectrumQ15;

void SIGNAL_TimeQ15ToSpectrumQ15(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumQ15 *freqData);
#endif

#ifdef SIGNAL_F32_ENABLE

typedef struct SIGNAL_SpectrumF32 {
  int points;        // number of points in the spectrum
  double sampleRate; // sample rate of the signal
  float *ampData;    // pointer to the amplitude data. Only valid before the
                     // next call to this function.
  float *cfftData;   // pointer to the complex FFT data. Only valid before the
                     // next call to this function.
  float dc;          // dc component
  double peakFreq;   // frequency of the peak
  float peakAmp;     // amplitude of the peak
} SIGNAL_SpectrumF32;

typedef struct SIGNAL_PeaksF32 {
  int count; // number of peaks found
  struct {
    double freq;             // frequency of the peak
    float amp;               // amplitude of the peak
  } peaks[SIGNAL_MAX_PEAKS]; // array of peaks
} SIGNAL_PeaksF32;

void SIGNAL_TimeQ15ToSpectrumF32(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumF32 *freqData);
void SIGNAL_TimeF32ToSpectrumF32(SIGNAL_TimeDataF32 *timeData,
                                 SIGNAL_SpectrumF32 *freqData);
#endif

void SIGNAL_FindPeaksF32(SIGNAL_SpectrumF32 *freqData,
                                 SIGNAL_PeaksF32 *peaks, float height,
                                 int distance);

// returns the correlation between two signals
double SIGNAL_GetCorrelationF32(const float *x, const float *y, int n);
// Unwraps the phase of a signal in place
void SIGNAL_UnwrapPhaseF32(float *phase, int n);