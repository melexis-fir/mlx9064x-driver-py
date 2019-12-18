"""
MptChannel class
"""

from mlx.pympt.core import *
import struct

DEBUG_PRINT = False


class MptChannel:
    """Abstract channel class"""
    def write(self, data):
        raise NotImplementedError()

    def read(self, nbytes):
        raise NotImplementedError()

    def flush_buffers(self):
        pass

    @staticmethod
    def mlx_crc(data, ):
        """Calculate checksum of bytes in data"""
        crc = 0
        for i in data:
            crc += i
            if crc > 255:
                crc -= 255
        return 255-crc

    def send_command(self, data):
        """Send a command

        This method prepares a command to be sent according to Melexis USB HID
        format. It adds number of bytes in front of the buffer and checksum at
        the end. Then buffer is sent as series of output reports, each with
        payload of 64 bytes.
        """
        n = len(data)
        if n < 1:
            return
        if n > 253:
            raise CommandTooLongException()

        crc = MptChannel.mlx_crc(data)

        # Add number of bytes (cmd + data) in front
        data = bytes([n]) + data

        # Add checksum at the end
        data += bytes([crc])

        # print (">SEND:", data)

        self.flush_buffers()
        self.write(data)

        answer = self._receive_answer()
        # print ("<RECV:", answer[:10]) # limit only to print first 10 bytes

        return answer

    def _receive_answer(self):
        """Receive an answer"""
        # read the first byte, specifying the length of the answer
        n = self.read(1)

        if len(n) == 0:
            raise ValueError("The channel did not return any data")

        if n[0] == 255:
            # Special case; n is 16 bits wide
            data = self.read(2)
            count = struct.unpack('>H', data)[0]

            crc_calc = 1
            crc_calc += data[0]
            if crc_calc > 255: crc_calc -= 255
            crc_calc += data[1]
            if crc_calc > 255: crc_calc -= 255

        else:
            count = n[0]
            crc_calc = 0

        # read the rest of the answer + checksum byte
        data = self.read(count + 1)
        #if DEBUG_PRINT:
        #    print(data)

        # verify the checksum
        crc = data[-1]
        data = data[:-1]

        for i in data:
            crc_calc += i
            if crc_calc > 255:
                crc_calc -= 255

        crc_calc = 255 - crc_calc

        if crc_calc != crc:
            raise BadCrcException()
        # TODO : check communication errors from device

        return data
