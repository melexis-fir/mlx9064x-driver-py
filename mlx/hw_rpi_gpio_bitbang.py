import RPi.GPIO as GPIO
import time
from mlx.hw_i2c_hal import HwI2cHalMlx90640


class HwRpiGpioBitBang(HwI2cHalMlx90640):
    support_buffer = False
    pass

    def __init__(self, sda=3, scl=5):
        if isinstance(sda, str) and sda.startswith ("I2CBB-"):
            scl = int(sda[9:11])
            sda = int(sda[6:8])
        self.i2c = I2cBitBang(sda, scl)
        self.i2c.reset()
        time.sleep(5000/1000000)

    def connect(self):
        self.i2c.reset()

    def i2c_read(self, i2c_addr, addr, count=2):
        data = []

        self.i2c.start()
        self.i2c.send_byte(i2c_addr << 1 | I2cBitBang.WRITE)
        self.i2c.send_byte((addr & 0xFF00) >> 8)
        self.i2c.send_byte(addr & 0x00FF)
        self.i2c.start()
        self.i2c.send_byte(i2c_addr << 1 | I2cBitBang.READ)

        for i in range(count // 2):
            data.append(self.i2c.read_byte())
            self.i2c.send_bit(I2cBitBang.ACK)
            data.append(self.i2c.read_byte())
            self.i2c.send_bit(I2cBitBang.ACK)

        self.i2c.stop()
        return bytes(data), 0

    def i2c_write(self, i2c_addr, addr, data):
        self.i2c.start()
        self.i2c.send_byte(i2c_addr << 1 | I2cBitBang.WRITE)
        self.i2c.send_byte((addr & 0xFF00) >> 8)
        self.i2c.send_byte(addr & 0x00FF)
        for d in data:
            self.i2c.send_byte(d)
        self.i2c.stop()
        return 0

    def get_hardware_id(self):
        return "Raspberry Pi I2C GPIO BitBang"


class I2cBitBang:
    WRITE = 0
    READ = 1
    ACK = 0

    def __init__(self, sda, scl, freq=400000):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.cleanup ([sda, scl])
        GPIO.setmode(GPIO.BOARD)
        self.freq = freq
        self.scl = scl
        self.sda = sda
        GPIO.setwarnings(False)
        GPIO.setup(self.sda, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.scl, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.reset()

    ''' Reads a bit from sda '''
    def read_bit(self):
        self.release(self.sda)
        self.release(self.scl)
        s = GPIO.input(self.sda)
        self.pull(self.scl)
        self.pull(self.sda)

        return s

    ''' Reads a byte, MSB first '''
    def read_byte(self):
        byte = 0x00

        for i in range(8):
            byte = (byte << 1) | self.read_bit()

        return byte

    ''' Sends 0 or 1:
    * Clock down, send bit, clock up, wait, clock down again
    * In clock stretching, slave holds the clock line down in order
    * to force master to wait before send more data '''
    def send_bit(self, bit):
        if bit:
            self.release(self.sda)
        else:
            self.pull(self.sda)

        self.release_wait(self.scl)
        self.pull(self.scl)

        self.pull(self.sda)

    ''' Sends 8 bit in a row, MSB first and reads ACK.
    * Returns I2C_ACK if device ack'ed '''
    def send_byte(self, byte):
        for i in range(8):
            self.send_bit(byte & 0x80)
            byte = byte << 1

        return self.read_bit()

    ''' In case of clock stretching or busy bus we must wait '''
    @staticmethod
    def release_wait(pin):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # time.sleep((1/self.freq)/2)

        while not GPIO.input(pin):
            time.sleep(1 / 1000000)

        # time.sleep((1/self.freq)/2)

    ''' Start: pull SDA while SCL is up '''
    ''' Best practice is to ensure the bus is not busy before start '''
    def start(self):
        if not self.release(self.sda):
            self.reset()
        self.release_wait(self.scl)

        self.pull(self.sda)
        self.pull(self.scl)

    ''' Stop: release SDA while SCL is up '''
    def stop(self):
        self.release_wait(self.scl)
        if not self.release(self.sda):
            self.reset()

    ''' Release: releases the line and return line status '''
    def release(self, pin):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # time.sleep((1/self.freq)/2)

        return GPIO.input(pin)

    ''' Pull: drives the line to level LOW '''
    def pull(self, pin):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        # time.sleep((1/self.freq)/2)

    def reset(self):
        for i in range(9):
            self.pull(self.scl)
            self.release(self.scl)
            if GPIO.input(self.sda):
                break

        self.pull(self.scl)
        self.pull(self.sda)
        self.stop()


def main ():
    i2c = I2cBitBang(3,5)

    for i2c_addr in range (127):
        i2c.start ()
        ack = i2c.send_byte (i2c_addr << 1 | I2cBitBang.READ)
        if ack == 0:
            print ("0x{:02X} has ACK!".format(i2c_addr))


if __name__ == '__main__':
    main()
