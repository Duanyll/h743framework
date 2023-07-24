#include <stdint.h>

typedef struct NN_SignetInferenceInstance {
  /**
   * Buffer for forward pass of the network. Allocate memory for this buffer.
   * slot[0]: at least max(cin * 143, 8 * 70) * sizeof(float) bytes
   * slot[1]: at least 16 * 8 * 16 * sizeof(float) bytes
   * slot[2]: at least 16 * sizeof(float) bytes
   * slot[3]: at least 8 * 70 * sizeof(float) bytes
   * Total: 11568 bytes
   */
  float *slots[4];
} NN_SignetInferenceInstance;

void NN_SignetInstance_init(NN_SignetInferenceInstance *instance,
                            uint8_t *buffer);

void NN_Signet_Evaluate(NN_SignetInferenceInstance *instance, float *input,
                        float *output);