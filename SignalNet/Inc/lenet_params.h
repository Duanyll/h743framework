#pragma once
#include <stdint.h>
#define NN_Lenet_Input_Channels 1
#define NN_Lenet_Output_Classes 10
// layers.conv1.weight, torch.Size([6, 1, 5, 5])
extern const uint32_t NN_Lenet_Param_layers_conv1_weight_Size;
// layers.conv1.weight, torch.Size([6, 1, 5, 5])
extern const uint32_t NN_Lenet_Param_layers_conv1_weight_Data[];
// layers.conv1.bias, torch.Size([6])
extern const uint32_t NN_Lenet_Param_layers_conv1_bias_Size;
// layers.conv1.bias, torch.Size([6])
extern const uint32_t NN_Lenet_Param_layers_conv1_bias_Data[];
// layers.conv2.weight, torch.Size([16, 6, 5, 5])
extern const uint32_t NN_Lenet_Param_layers_conv2_weight_Size;
// layers.conv2.weight, torch.Size([16, 6, 5, 5])
extern const uint32_t NN_Lenet_Param_layers_conv2_weight_Data[];
// layers.conv2.bias, torch.Size([16])
extern const uint32_t NN_Lenet_Param_layers_conv2_bias_Size;
// layers.conv2.bias, torch.Size([16])
extern const uint32_t NN_Lenet_Param_layers_conv2_bias_Data[];
// layers.fc1.weight, torch.Size([120, 256])
extern const uint32_t NN_Lenet_Param_layers_fc1_weight_Size;
// layers.fc1.weight, torch.Size([120, 256])
extern const uint32_t NN_Lenet_Param_layers_fc1_weight_Data[];
// layers.fc1.bias, torch.Size([120])
extern const uint32_t NN_Lenet_Param_layers_fc1_bias_Size;
// layers.fc1.bias, torch.Size([120])
extern const uint32_t NN_Lenet_Param_layers_fc1_bias_Data[];
// layers.fc2.weight, torch.Size([84, 120])
extern const uint32_t NN_Lenet_Param_layers_fc2_weight_Size;
// layers.fc2.weight, torch.Size([84, 120])
extern const uint32_t NN_Lenet_Param_layers_fc2_weight_Data[];
// layers.fc2.bias, torch.Size([84])
extern const uint32_t NN_Lenet_Param_layers_fc2_bias_Size;
// layers.fc2.bias, torch.Size([84])
extern const uint32_t NN_Lenet_Param_layers_fc2_bias_Data[];
// layers.fc3.weight, torch.Size([10, 84])
extern const uint32_t NN_Lenet_Param_layers_fc3_weight_Size;
// layers.fc3.weight, torch.Size([10, 84])
extern const uint32_t NN_Lenet_Param_layers_fc3_weight_Data[];
// layers.fc3.bias, torch.Size([10])
extern const uint32_t NN_Lenet_Param_layers_fc3_bias_Size;
// layers.fc3.bias, torch.Size([10])
extern const uint32_t NN_Lenet_Param_layers_fc3_bias_Data[];
