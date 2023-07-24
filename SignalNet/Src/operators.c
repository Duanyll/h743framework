#include "operators.h"

#include <math.h>

/// @brief 1D convolution with kernel size 3, stride 1, padding 1 and relu
/// activation function
/// @param pIn (Cin x L)
/// @param pWeight (Cout x Cin x 3)
/// @param pBias (Cout)
/// @param pOut (Cout x L)
/// @param inChannel Cin
/// @param outChannel Cout
/// @param inLength L
void NN_Conv1d_k3s1p1_relu_f32(float *pIn, float *pWeight, float *pBias,
                               float *pOut, int inChannel, int outChannel,
                               int inLength) {
  for (int oc = 0; oc < outChannel; oc++) {
    for (int ol = 0; ol < inLength; ol++) {
      float sum = 0;
      for (int ic = 0; ic < inChannel; ic++) {
        for (int kl = 0; kl < 3; kl++) {
          int il = ol + kl - 1;
          if (il < 0 || il >= inLength) {
            continue;
          }
          sum += pIn[ic * inLength + il] *
                 pWeight[oc * inChannel * 3 + ic * 3 + kl];
        }
      }
      sum += pBias[oc];
      pOut[oc * inLength + ol] = sum > 0 ? sum : 0;
    }
  }
}

/// @brief 1D convolution with kernel size 5, stride 2, padding 0 and relu
/// activation function
/// @param pIn (Cin x Lin)
/// @param pWeight (Cout x Cin x 5)
/// @param pBias (Cout)
/// @param pOut (Cout x Lout)
/// @param inChannel Cin
/// @param outChannel Cout
/// @param inLength Lin (Lout = (Lin - 5) / 2 + 1)
void NN_Conv1d_k5s2p0_relu_f32(float *pIn, float *pWeight, float *pBias,
                               float *pOut, int inChannel, int outChannel,
                               int inLength) {
  int outLength = (inLength - 5) / 2 + 1;
  for (int oc = 0; oc < outChannel; oc++) {
    for (int ol = 0; ol < outLength; ol++) {
      float sum = 0;
      for (int ic = 0; ic < inChannel; ic++) {
        for (int kl = 0; kl < 5; kl++) {
          int il = ol * 2 + kl;
          sum += pIn[ic * inLength + il] *
                 pWeight[oc * inChannel * 5 + ic * 5 + kl];
        }
      }
      sum += pBias[oc];
      pOut[oc * outLength + ol] = sum > 0 ? sum : 0;
    }
  }
}

/// @brief 1d average pooling with kernel size 2, stride 2, padding 0
/// @param pIn (Cin x Lin)
/// @param pOut (Cin x Lout)
/// @param inChannel Cin
/// @param inLength Lin (Lout = (Lin - 2) / 2 + 1)
void NN_AvgPool1d_k2_f32(float *pIn, float *pOut, int inChannel, int inLength) {
  int outLength = (inLength - 2) / 2 + 1;
  for (int ic = 0; ic < inChannel; ic++) {
    for (int ol = 0; ol < outLength; ol++) {
      pOut[ic * outLength + ol] =
          (pIn[ic * inLength + ol * 2] + pIn[ic * inLength + ol * 2 + 1]) / 2;
    }
  }
}

/// @brief 1d average pooling with kernel size 4, stride 4, padding 0
/// @param pIn (Cin x Lin)
/// @param pOut (Cin x Lout)
/// @param inChannel Cin
/// @param inLength Lin (Lout = (Lin - 2) / 2 + 1)
void NN_AvgPool1d_k4_f32(float *pIn, float *pOut, int inChannel, int inLength) {
  int outLength = (inLength - 4) / 4 + 1;
  for (int ic = 0; ic < inChannel; ic++) {
    for (int ol = 0; ol < outLength; ol++) {
      pOut[ic * outLength + ol] =
          (pIn[ic * inLength + ol * 4] + pIn[ic * inLength + ol * 4 + 1] +
           pIn[ic * inLength + ol * 4 + 2] + pIn[ic * inLength + ol * 4 + 3]) /
          4;
    }
  }
}

/// @brief Linear layer with relu activation function
/// @param pIn (Fin)
/// @param pWeight (Fout x Fin)
/// @param pBias (Fout)
/// @param pOut (Fout)
/// @param inFeature Fin
/// @param outFeature Fout
void NN_Linear_relu_f32(float *pIn, float *pWeight, float *pBias, float *pOut,
                        int inFeature, int outFeature) {
  for (int of = 0; of < outFeature; of++) {
    float sum = 0;
    for (int ife = 0; ife < inFeature; ife++) {
      sum += pIn[ife] * pWeight[of * inFeature + ife];
    }
    sum += pBias[of];
    pOut[of] = sum > 0 ? sum : 0;
  }
}

void NN_Linear_f32(float *pIn, float *pWeight, float *pBias, float *pOut,
                   int inFeature, int outFeature) {
  for (int of = 0; of < outFeature; of++) {
    float sum = 0;
    for (int ife = 0; ife < inFeature; ife++) {
      sum += pIn[ife] * pWeight[of * inFeature + ife];
    }
    sum += pBias[of];
    pOut[of] = sum;
  }
}

/// @brief Softmax function
/// @param pIn (F)
/// @param pOut (F)
/// @param inLength F
void NN_Softmax_f32(float *pIn, float *pOut, int inLength) {
  float sum = 0;
  for (int i = 0; i < inLength; i++) {
    sum += expf(pIn[i]);
  }
  for (int i = 0; i < inLength; i++) {
    pOut[i] = expf(pIn[i]) / sum;
  }
}

void NN_BatchNorm1d_f32(float *pIn, float *pOut, float *pWeight, float *pBias,
                        int inChannel, int inLength) {
  for (int ic = 0; ic < inChannel; ic++) {
    for (int il = 0; il < inLength; il++) {
      pOut[ic * inLength + il] =
          pIn[ic * inLength + il] * pWeight[ic] + pBias[ic];
    }
  }
}
void NN_BatchNorm1d_Inplace_f32(float *pIn, float *pWeight, float *pBias,
                                int inChannel, int inLength) {
  for (int ic = 0; ic < inChannel; ic++) {
    for (int il = 0; il < inLength; il++) {
      pIn[ic * inLength + il] =
          pIn[ic * inLength + il] * pWeight[ic] + pBias[ic];
    }
  }
}