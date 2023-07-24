#include "graph.h"
#include "operators.h"
#include "signet_params.h"

#include <string.h>

void NN_SignetInstance_init(NN_SignetInferenceInstance *instance,
                            uint8_t *buffer) {
  instance->slots[0] = (float *)buffer;
  instance->slots[1] = (float *)(buffer + 8 * 70 * sizeof(float));
  instance->slots[2] =
      (float *)(buffer + 8 * 70 * sizeof(float) + 16 * 8 * 16 * sizeof(float));
  instance->slots[3] =
      (float *)(buffer + 8 * 70 * sizeof(float) + 16 * 8 * 16 * sizeof(float) +
                16 * sizeof(float));
}

#define SLOT(id) instance->slots[id]
#define SIGNET_Load(name, id)                                                  \
  memcpy(instance->slots[id], NN_Signet_Param_layers_##name##_Data,            \
         NN_Signet_Param_layers_##name##_Size * sizeof(float))

void NN_Signet_Evaluate(NN_SignetInferenceInstance *instance, float *input,
                        float *output) {
  memcpy(SLOT(0), input,
         NN_Signet_Input_Samples * NN_Signet_Input_Channels * sizeof(float));
  SIGNET_Load(conv1_weight, 1);
  SIGNET_Load(conv1_bias, 2);
  NN_Conv1d_k5s2p0_relu_f32(SLOT(0), SLOT(1), SLOT(2), SLOT(3),
                            NN_Signet_Input_Channels, 8,
                            NN_Signet_Input_Samples);
  SIGNET_Load(bn1_weight, 1);
  SIGNET_Load(bn1_bias, 2);
  NN_BatchNorm1d_Inplace_f32(SLOT(3), SLOT(1), SLOT(2), 8, 70);
  NN_AvgPool1d_k2_f32(SLOT(3), SLOT(0), 8, 70);
  SIGNET_Load(conv2_weight, 1);
  SIGNET_Load(conv2_bias, 2);
  NN_Conv1d_k5s2p0_relu_f32(SLOT(0), SLOT(1), SLOT(2), SLOT(3), 8, 16, 35);
  SIGNET_Load(bn2_weight, 1);
  SIGNET_Load(bn2_bias, 2);
  NN_BatchNorm1d_Inplace_f32(SLOT(3), SLOT(1), SLOT(2), 16, 16);
  NN_AvgPool1d_k2_f32(SLOT(3), SLOT(0), 16, 16);
  SIGNET_Load(fc1_weight, 1);
  SIGNET_Load(fc1_bias, 2);
  NN_Linear_relu_f32(SLOT(0), SLOT(1), SLOT(2), SLOT(3), 16 * 8, 16);
  SIGNET_Load(fc2_weight, 1);
  SIGNET_Load(fc2_bias, 2);
  NN_Linear_f32(SLOT(3), SLOT(1), SLOT(2), SLOT(0), 16,
                NN_Signet_Output_Classes);
  NN_Softmax_f32(SLOT(0), output, NN_Signet_Output_Classes);
}