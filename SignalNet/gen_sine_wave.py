# generate sine waveforms
# 128 samples per cycle
# range: 0 to 2^16-1
# 16 bit resolution
# output to a C array

import math
import numpy as np

# number of samples per cycle
samples_per_cycle = 128

i = np.arange(samples_per_cycle, dtype=np.float32) / samples_per_cycle * 2 * math.pi
sine_wave = np.sin(i) * 32767 + 32767

print("const uint16_t sine_wave[128] = {")
for i in range(samples_per_cycle):
    print("    %d," % sine_wave[i])
print("};")