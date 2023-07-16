import serial
import numpy as np
from tqdm import tqdm
import time
import pyvisa

def test_wave_gen():
    rm = pyvisa.ResourceManager()
    print(rm.list_resources())
    device_addr = input("Enter device address: ")
    device = rm.open_resource(device_addr)
    for i in range(1000, 10000, 1000):
        device.write(f"C1:BSWV FRQ,{i}")
        time.sleep(0.5)

def main():
    com_port_name = input("Enter COM port name: ")
    com = serial.Serial(
        port=com_port_name,
        baudrate=115200,
        bytesize=8,
        timeout=None,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    rm = pyvisa.ResourceManager()
    devices = rm.list_resources()
    if (len(devices) == 1):
        device = rm.open_resource(devices[0])
    else:
        print(devices)
        device_addr = input("Enter device address: ")
        device = rm.open_resource(device_addr)

    params = np.load("params_fm.npy")
    length = params.shape[0] 
    # length = 20   

    data_length = 512
    data = np.zeros((0, data_length))
    try:
        for i in tqdm(range(length)):
            device.write(f"C1:BSWV AMP,{0.05}")
            device.write(f"C1:MDWV FM,FRQ,{params[i, 1]}")
            device.write(f"C1:MDWV FM,DEVI,{params[i, 2]}")
            time.sleep(0.1)

            com.write(b"\x01\xff\xff\xff")
            com.read_until(b"\xff\xff\xff\xff")
            buf1 = com.read(data_length * 4)
            arr1 = np.frombuffer(buf1, dtype=np.float32)
            com.read_until(b"\xff\xff\xff\xff")
            buf2 = com.read(data_length * 4)
            arr2 = np.frombuffer(buf2, dtype=np.float32)
            data = np.vstack((data, arr1, arr2))
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        com.close()
        np.save("data.npy", data)

if __name__ == "__main__":
    main()
    # test_wave_gen()