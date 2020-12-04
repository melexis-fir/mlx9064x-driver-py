from mlx.hw_i2c_hal import HwI2cHalMlx90640
import mlx.pympt as pympt
from mlx.pympt.core import *

import serial.tools.list_ports
import sys
import time
import struct
from math import ceil


USB_VID = 1001
USB_PID = 32


class Mlx90640Commands:
    CMD_ResetHardware = bytes([0])
    CMD_GetHardwareID = bytes([1])
    CMD_GetSoftwareID = bytes([3])
    CMD_ReadEEPROM = bytes([10])
    CMD_I2C_Master_SW = bytes([31])
    CMD_SetVdd = bytes([150])
    CMD_StopDAQ = bytes([152])
    CMD_ReadDAQ = bytes([153])
    CMD_MeasureVddIdd = bytes([156])
    CMD_StartDAQ_90640 = bytes([171])
    CMD_I2C_Master_90640 = bytes([174])
    CMD_StartDAQ_90641 = bytes([179])


class HwUsbEvb90640(HwI2cHalMlx90640):
    __command_response_pairs = {
        #  name: (command to send, expected response)               byte count and crc not included
        "init_SW_I2C": ([0x1E, 2, 0, 0, 0, 6, 0, 0, 0, 8, 0, 0, 0, 5, 0, 0, 0], [0x1E]),
        "begin_conversion": ([0xAE, 0x33, 0x80, 0x00, 0x00, 0x20, 0x00, 0x00], [0xAE, 0x00]),
    }

    def __init__(self, comport=None):
        self.support_buffer = True
        self.frames_buffer = []
        self.m_lDaqFrameIdx = 0
        self.frame_length_bytes = None

        self.comport = comport
        if comport is None or comport == 'auto':
            comports = HwUsbEvb90640.list_serial_ports(USB_PID, USB_VID)
            if len(comports) < 1:
                raise ValueError("no EVB90640 found; please connect to USB port")
            if len(comports) > 1:
                print("WARN: found more than one EVB90640 {}, using the first one".format(comports))
            self.comport = comports[0]

        print("comport = {}".format (self.comport))
        self.channel = pympt.UsbSerialChannel()
        self.channel.connect(self.comport)

    @staticmethod
    def list_serial_ports(pid=None, vid=None):
        """ Lists serial port names which startswith serial_number like in argument.

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
        """
        ports = []
        if sys.platform.startswith('win') or sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            for sp in serial.tools.list_ports.comports():
                if pid is None or vid is None:
                    ports.append(str(sp.device))
                else:
                    if sp.vid == vid and sp.pid == pid:
                        ports.append(str(sp.device))
            return ports
        elif sys.platform.startswith('darwin'):
            import glob
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def __send_buffered_command(self, name):
        """this function should be removed in the future"""
        resp = self.channel.send_command(bytes(HwUsbEvb90640.__command_response_pairs[name][0]))
        if list(bytes(resp)) != HwUsbEvb90640.__command_response_pairs[name][1]:
            raise ValueError("Did not get expected response from EVB, {} != {}"
                             .format(HwUsbEvb90640.__command_response_pairs[name][1], list(bytes(resp))))

    def connect(self):
        """
        Do the necessary initialisation before measurements are taken

        procedure:
            poll until HW id received (timeout??)
            init SW I2C
            set vdd
            I2C:
                set refresh rate
                start conversion
            set evb refresh rate
        """

        timed_out = True
        for i in range(5):
            try:
                if len(self.get_hardware_id()) != 0:
                    timed_out = False
                    break
            except pympt.MptException:
                pass
            time.sleep(1)

        if timed_out:
            raise NotConnectedException("The command timed out while attempting to get HW id")

        # TODO move out of function
        self.set_vdd(3.3)
        self.__send_buffered_command("init_SW_I2C")
        self.__send_buffered_command("begin_conversion")
        self.channel.send_command(bytes([0xAE, 0x33, 0x24, 0x00, 0x80, 0x06]))
        self.channel.send_command(bytes([0xAE, 0x33, 0x80, 0x00, 0x22, 0x00]))

        self.get_sensor_type(0x33)
        if self.sensor_type == 0:
            self.frame_length_bytes = 32 * 26 * 2
        if self.sensor_type == 1:
            self.frame_length_bytes = 16 * 16 * 2

    def read_frame(self, i2c_addr):
        if self.frames_buffer is not None and len(self.frames_buffer) > 0:
            return self.frames_buffer.pop(0)
        self.frames_buffer = self.read_frames()
        if self.frames_buffer is None:
            return None
        return self.frames_buffer.pop(0)

    def start_data_acquisition(self, i2c_addr, frame_rate_hz):
        wait_us = ceil(1000000 / frame_rate_hz)
        if self.sensor_type == 0:
            cmd = Mlx90640Commands.CMD_StartDAQ_90640 + struct.pack("<BL", i2c_addr, wait_us)
            result = self.channel.send_command(cmd)
            if result != bytes([0xAB, 0x00]):
                raise Exception("Error during execution of command on the EVB")
        else:
            cmd = Mlx90640Commands.CMD_StartDAQ_90641 + struct.pack("<BL", i2c_addr, wait_us)
            result = self.channel.send_command(cmd)
            if result != bytes([0xB3, 0x00]):
                raise Exception("Error during execution of command on the EVB")

    def get_hardware_id(self):
        return self.channel.send_command(Mlx90640Commands.CMD_GetHardwareID)

    def set_vdd(self, vdd):
        """Set Vdd of the sensor"""
        cmd = Mlx90640Commands.CMD_SetVdd + bytes([0])
        cmd = cmd + struct.pack("<f", float(vdd))
        self.channel.send_command(cmd)

    def read_frames(self):
        """
        Sends a read request to the EVB. If the EVB has buffered any frames (the EVB will buffer up to 4 frames)
        a list of them is returned. If no frames have been buffered None will be returned.
        a frame is an array with size 32 * 26 containing signed 16 bit integers.

        :return: list of frames (each frame is a list of 32 * 26 signed 16 bit ints) or None
        :raises: ValueError - data received from EVB is not at the expected length, or no data is received
        """
        data = self.channel.send_command(Mlx90640Commands.CMD_ReadDAQ + bytes([0]))
        if data[1] != 0:
            raise ValueError("EVB90640: EVB Frame buffer full")

        frames = None
        received_data_len = len(data) - 2
        if received_data_len >= self.frame_length_bytes:
            # one or more frames have been received
            if received_data_len % self.frame_length_bytes != 0:
                raise ValueError("Invalid data length from EVB")

            frames = []
            for i in range(received_data_len // self.frame_length_bytes):
                self.m_lDaqFrameIdx += 1
                frame_data_start = 2 + i * self.frame_length_bytes
                frame_data_end = 2 + (i + 1) * self.frame_length_bytes
                frame_data = data[frame_data_start: frame_data_end]
                frame = list(struct.unpack(">" + str(self.frame_length_bytes // 2) + "h", frame_data))
                frames.append(frame)

        return frames

    def i2c_read(self, i2c_addr, addr, count=1):
        """
        Read consecutive data from the device
        :param int i2c_addr: the I2C slave address of the MLX9064x
        :param int addr: the start address of the data
        :param int count: the number of consecutive bytes to be read
        :return: tuple(result_data[], acknowledge_errors)
        """
        cmd = Mlx90640Commands.CMD_I2C_Master_90640 + \
            struct.pack(">BH", i2c_addr, addr) + struct.pack("<H", count)
        result = self.channel.send_command(cmd)
        return result[2:], result[1]

    def i2c_write(self, i2c_addr, addr, data):
        """
        Write consecutive data to the device
        :param int i2c_addr: the I2C slave address of the MLX9064x
        :param int addr: the start address of the data
        :param bytes data: data to send
        :return: acknowledge_errors
        """
        cmd = Mlx90640Commands.CMD_I2C_Master_90640 + \
            struct.pack(">BH", i2c_addr, addr) + data + struct.pack("H", 0)
        result = self.channel.send_command(cmd)
        return result[1]

    def clear_error(self, i2c_addr, frame_rate_hz):
        self.channel.send_command(Mlx90640Commands.CMD_StopDAQ)
        self.set_vdd(3.3)
        wait_us = ceil(1000000 / frame_rate_hz)
        cmd = Mlx90640Commands.CMD_StartDAQ_90640 + struct.pack("<BL", i2c_addr, wait_us)
        self.channel.send_command(cmd)

    def measure_vdd(self):
        """
        Measure Vdd of the sensor

        @:return vdd
        """
        data = self.channel.send_command(Mlx90640Commands.CMD_MeasureVddIdd)
        vdd = struct.unpack('<f', data[1:5])[0]
        return vdd
