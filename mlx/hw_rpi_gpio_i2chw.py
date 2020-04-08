from smbus2 import SMBus, i2c_msg
import time
import struct
import os
from mlx.hw_i2c_hal import HwI2cHalMlx90640
import platform


class HwRpiI2cHw(HwI2cHalMlx90640):
    support_buffer = False
    pass

    def __init__(self, channel=1):
        if isinstance(channel, str) and channel.startswith ("I2C-"):
            channel = int(channel[4:])

        if channel == 1 and platform.machine().startswith('armv'):
            os.system('raspi-gpio set 2 a0')
            os.system('raspi-gpio set 3 a0')
        self.i2c = SMBus(channel)

    def connect(self):
        pass

    def i2c_read(self, i2c_addr, addr, count=2):
        addr_msb = addr >> 8 & 0x00FF
        addr_lsb = addr & 0x00FF

        write = i2c_msg.write(i2c_addr, [addr_msb, addr_lsb])
        read = i2c_msg.read(i2c_addr, count)
        self.i2c.i2c_rdwr(write, read)
        return bytes(list(read)), 0

    def i2c_write(self, i2c_addr, addr, data):
        cmd = []
        reg_msb = addr >> 8
        cmd.append(addr & 0x00FF)

        for d in data:
          cmd.append(d)
        self.i2c.write_i2c_block_data(i2c_addr, reg_msb, cmd)
        return 0

    def get_hardware_id(self):
        return "Raspberry Pi I2C Hardware"


def main():
    i2c = HwRpiI2cHw()
    d = i2c.i2c_read (0x33, 0x2400, 10) # example to read 10 first bytes...
    print(d)

    for i2c_addr in range(127):
        try:
            read_data = i2c.i2c.read_byte_data(i2c_addr, 0)
        except Exception as e:
            pass
        else:
            print ("0x{:02X} => ACK!".format (i2c_addr))


if __name__ == '__main__':
    main()
