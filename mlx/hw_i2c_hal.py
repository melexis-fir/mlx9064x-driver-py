import struct

class HwI2cHalMlx90640:
    support_buffer = False
    sensor_type = None
    pass

    def connect(self):
        print("connect function not implemented")

    def i2c_read(self, i2c_addr, addr, count=1):
        print("i2c_read function not implemented")
        return bytes([0] * count), 0

    def i2c_write(self, i2c_addr, addr, data):
        print("i2c_write function not implemented")
        return 0

    def get_sensor_type(self, i2c_addr):
        sensor, stat = self.i2c_read(i2c_addr, 0x240A, 2)
        sensor = struct.unpack(">H",sensor)[0]
        self.sensor_type = (sensor & 0x40) >> 6
        return self.sensor_type

    def read_frame(self, i2c_addr):
        # 1. wait until new data is available
        # 2. read frame data
        # 3. clear new data available bit.

        # 1. wait until new data is available
        new_data_available = False
        sub_page = 0
        while not new_data_available:
            status_reg, status = self.i2c_read(i2c_addr, 0x8000, 2)
            status_reg = struct.unpack(">H", status_reg[0:2])[0]
            if status_reg & 0x0008:
                new_data_available = True
            sub_page = status_reg & 0x0001

        # 2. read frame data
        self.i2c_write(i2c_addr, 0x8000, struct.pack("<H", 0x0030))

        if self.sensor_type is None:
            self.get_sensor_type(i2c_addr)
        if self.sensor_type == 0:
            frame_data, status = self.i2c_read(i2c_addr, 0x0400, 832*2)  # 32 * 26 * 2
            frame_data = list(struct.unpack(">832h", frame_data))
        else:
            frame_data, status = self.i2c_read(i2c_addr, 0x0400, 256*2)  # 16 * 16 * 2
            frame_data = list(struct.unpack(">256h", frame_data))

        # 3. clear new data available bit.
        self.i2c_write(i2c_addr, 0x8000, struct.pack("<H", status_reg & ~0x0008))

        control_reg1, status = self.i2c_read(i2c_addr, 0x800D, 2)
        control_reg1 = struct.unpack(">H", control_reg1[0:2])[0]

        return frame_data + [control_reg1, status_reg]

    def get_hardware_id(self):
        return "HardWare Abstraction Layer (dummy)"
