#pragma once

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
                               int inLength);

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
                               int inLength);

/// @brief 1d average pooling with kernel size 2, stride 2, padding 0
/// @param pIn (Cin x Lin)
/// @param pOut (Cin x Lout)
/// @param inChannel Cin
/// @param inLength Lin (Lout = (Lin - 2) / 2 + 1)
void NN_AvgPool1d_k2_f32(float *pIn, float *pOut, int inChannel, int inLength);

/// @brief 1d average pooling with kernel size 4, stride 4, padding 0
/// @param pIn (Cin x Lin)
/// @param pOut (Cin x Lout)
/// @param inChannel Cin
/// @param inLength Lin (Lout = (Lin - 2) / 2 + 1)
void NN_AvgPool1d_k4_f32(float *pIn, float *pOut, int inChannel, int inLength);

/// @brief Linear layer with relu activation function
/// @param pIn (Fin)
/// @param pWeight (Fout x Fin)
/// @param pBias (Fout)
/// @param pOut (Fout)
/// @param inFeature Fin
/// @param outFeature Fout
void NN_Linear_relu_f32(float *pIn, float *pWeight, float *pBias, float *pOut,
                        int inFeature, int outFeature);

void NN_Linear_f32(float *pIn, float *pWeight, float *pBias, float *pOut,
                   int inFeature, int outFeature);

/// @brief Softmax function
/// @param pIn (F)
/// @param pOut (F)
/// @param inLength F
void NN_Softmax_f32(float *pIn, float *pOut, int inLength);

void NN_BatchNorm1d_f32(float *pIn, float *pOut, float *pWeight, float *pBias,
                        int inChannel, int inLength);
void NN_BatchNorm1d_Inplace_f32(float *pIn, float *pWeight, float *pBias,
                                int inChannel, int inLength);
