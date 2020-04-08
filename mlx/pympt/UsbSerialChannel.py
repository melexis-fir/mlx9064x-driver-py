
from mlx.pympt.channel import *
import serial

DEBUG_PRINT=0

class UsbSerialChannel(MptChannel):
    def __init__(self):
        self.serial_con = None

    def connect(self, port, **serial_args):
        """Initialize USB Serial connection to EVB90640

        :raises ValueError: Will be raised when parameter are out of range, e.g. baud rate, data bits.
        :raises SerialException: In case the device can not be found or can not be configured.
        :returns if the connection was successful
        """
        if 'timeout' not in serial_args: serial_args['timeout'] = 10
        self.serial_con = serial.serial_for_url(port, **serial_args)
        if not self.serial_con.is_open:
            self.serial_con = None
            return False

        return True

    def disconnect(self):
        if self.serial_con is not None:
            self.serial_con.close()
        else:
            raise NotConnectedException()

    def flush_buffers(self):
        """
        Reset the buffers discarding any content in them
        """
        if self.serial_con is not None:
            self.serial_con.reset_input_buffer()
            self.serial_con.reset_output_buffer()
        else:
            raise NotConnectedException()

    def write(self, data):
        """
        :param data: data to be sent
        :return: number of bytes written
        :raises NotConnectedException - if a write attempt is done before a connect
        """
        if self.serial_con is not None:
            if DEBUG_PRINT:
                print('Sending: {}'.format(list(data)))

            return self.serial_con.write(data)
        else:
            raise NotConnectedException()

    def read(self, nbytes):
        if self.serial_con is not None:
            data = self.serial_con.read(nbytes)
            if DEBUG_PRINT:
                print('Receiving: {}'.format(list(data)))
            return data
        else:
            raise NotConnectedException()
