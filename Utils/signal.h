#pragma once

#include "main.h"
#include "utils_config.h"
#include "common.h"

typedef struct SIGNAL_TimeDataQ15 {
  int points;        // number of points in the signal
  int offset;        // offset of the time data in the buffer. used to handle
                     // multiple signals in the same buffer.
  int stride;        // stride of the time data in the buffer. used to handle
                     // multiple signals in the same buffer.
  double sampleRate; // sample rate of the signal
  int16_t *timeData; // pointer to the time data. by setting offset and
                     // stride, multiple signals can be stored in the
                     // same buffer.
  float range;       // range of the signal +/- range
  BOOL stripDc;      // if true, strip the DC component
} SIGNAL_TimeDataQ15;

typedef struct SIGNAL_TimeDataF32 {
  int points;
  int offset;
  int stride;
  double sampleRate;
  float *timeData;
  BOOL stripDc; // if true, strip the DC component
} SIGNAL_TimeDataF32;

#if defined(ARM_MATH_CM3) || defined(ARM_MATH_CM4) || defined(ARM_MATH_CM7) || \
    defined(ARM_MATH_CM0) || defined(ARM_MATH_CM0PLUS)

#include "arm_math.h"

typedef struct SIGNAL_FFTBufferF32 {
  arm_cfft_radix2_instance_f32 rad2Instance;
  arm_cfft_radix4_instance_f32 rad4Instance;
  float *fftBuffer; // Allocate at least points * 2 elements
  float *magBuffer; // Allocate at least points elements
} SIGNAL_FFTBufferF32;

#endif

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

#if defined(ARM_MATH_CM3) || defined(ARM_MATH_CM4) || defined(ARM_MATH_CM7) || \
    defined(ARM_MATH_CM0) || defined(ARM_MATH_CM0PLUS)

void SIGNAL_TimeQ15ToSpectrumF32(SIGNAL_TimeDataQ15 *timeData,
                                 SIGNAL_SpectrumF32 *freqData,
                                 SIGNAL_FFTBufferF32 *buffer);
void SIGNAL_TimeF32ToSpectrumF32(SIGNAL_TimeDataF32 *timeData,
                                 SIGNAL_SpectrumF32 *freqData,
                                 SIGNAL_FFTBufferF32 *buffer);

#endif

void SIGNAL_FindPeaksF32(SIGNAL_SpectrumF32 *freqData, SIGNAL_PeaksF32 *peaks,
                         float height, int distance);

// returns the correlation between two signals
double SIGNAL_GetCorrelationF32(const float *x, const float *y, int n);
// Unwraps the phase of a signal in place
void SIGNAL_UnwrapPhaseF32(float *phase, int n);

double SIGNAL_SimpleSNR(SIGNAL_SpectrumF32 *freqData);