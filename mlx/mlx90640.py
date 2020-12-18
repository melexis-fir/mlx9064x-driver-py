"""A python interface for Mlx9064x and MLX90641."""
import struct
import enum
from ctypes import *

from statistics import median
from math import fabs


class I2CAcknowledgeError(Exception):
    pass


class Mlx9064x:
    DAQ_CONT_16x12 = 5
    MIN_TEMP_DEGC = -273.15
    LSB_DEGC = 50.0

    def __init__(self, hw, i2c_addr=0x33, frame_rate=2.0):
        self.hw = None
        if hw is None:
            hw = 'auto'
        if isinstance(hw, str):
            if hw == 'auto' or hw.startswith("COM") or hw.startswith("com") or hw.startswith("/dev/tty"):
                from mlx.hw_usb_evb90640 import HwUsbEvb90640
                self.hw = HwUsbEvb90640(hw)
            if hw.startswith("I2C-"):
                from mlx.hw_rpi_gpio_i2chw import HwRpiI2cHw
                self.hw = HwRpiI2cHw(hw)
            if hw.startswith("I2CBB-"):
                from mlx.hw_rpi_gpio_bitbang import HwRpiGpioBitBang
                self.hw = HwRpiGpioBitBang(hw)
        else:
            self.hw = hw
        self.i2c_addr = i2c_addr
        self.calc_params = TCalcParams()
        self.m_lFilterTgcDepth = 8
        self.m_arrLastTgc = [[0] * TCalcParams.NUM_PAGES] * TCalcParams.NUM_TGC
        self.emissivity = 1.0

        self.hw.connect()
        self.frame_rate = frame_rate
        if self.hw.get_sensor_type(0x33) == 0:
            self.frame_length_bytes = 32 * 26 * 2
            self.eeprom = Mlx90640EEPROM(self)
        else:
            self.frame_length_bytes = 16 * 16 * 2
            self.eeprom = Mlx90641EEPROM(self)

    def init(self):
        self.eeprom.read_eeprom_from_device()
        self.calculate_parameters()

    def set_vdd(self, vdd):
        """Set Vdd of the sensor"""
        # if supported...
        if callable(getattr(self.hw, 'set_vdd', None)):
            self.hw.set_vdd(vdd)

    @property
    def frame_rate(self):
        return self.__frame_rate

    @frame_rate.setter
    def frame_rate(self, frame_rate):
        """
        Set the refresh rate of the camera
        :param float frame_rate: the new frame rate for the camera
        """
        # set the refresh rate on the chip
        ctrl_reg_1, status = self.hw.i2c_read(self.i2c_addr, 0x800D, 2)
        if 0 != status:
            raise I2CAcknowledgeError("Error during read of Control register 1")
        ctrl_reg_1 = struct.unpack(">H", ctrl_reg_1)[0]  # TODO find endian

        frame_rate_code = Mlx9064x.frame_rate_to_bit_mask(frame_rate)
        if frame_rate_code is None:
            raise ValueError("Invalid value for frame rate: {}; valid values are {}"
                             .format(frame_rate, [0.5, 1, 2, 4, 8, 16, 32, 64]))
        else:
            ctrl_reg_1 &= 0xFC7F  # clear the 3 bits that represent the frame rate
            ctrl_reg_1 |= frame_rate_code << 7

        status = self.hw.i2c_write(self.i2c_addr, 0x800D, struct.pack(">H", ctrl_reg_1))
        if 0 != status:
            raise I2CAcknowledgeError("Error during write of Control register 1")
        self.__frame_rate = frame_rate

        self.set_vdd(3.3)
        if self.hw.support_buffer:
            self.hw.start_data_acquisition(self.i2c_addr, frame_rate)

    def clear_error(self, frame_rate_hz):
        if callable(getattr(self.hw, 'clear_error', None)):
            self.hw.clear_error(self.i2c_addr, frame_rate_hz)

    @staticmethod
    def frame_rate_to_bit_mask(f_rate):
        frame_rates = [0.5, 1, 2, 4, 8, 16, 32, 64]
        if f_rate in frame_rates:
            return frame_rates.index(f_rate)
        else:
            return None

    def measure_vdd(self):
        """
        Measure Vdd of the sensor

        @:return vdd
        """
        if callable(getattr(self.hw, 'measure_vdd', None)):
            return self.hw.measure_vdd()
        return None

    def get_hardware_id(self):
        """
        :return: bytes array representing the hardware id
        """
        return self.hw.get_hardware_id()

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
        self.hw.connect()

    def read_frame(self):
        """
        Sends a read request to the EVB. If the EVB has buffered any frames (the EVB will buffer up to 4 frames)
        a list of them is returned. If no frames have been buffered None will be returned.
        a frame is an array with size 32 * 26 containing signed 16 bit integers.

        :return: list of frames (each frame is a list of 32 * 26 signed 16 bit ints) or None
        :raises: ValueError - data received from EVB is not at the expected length, or no data is received
        """
        return self.hw.read_frame(self.i2c_addr)

    def i2c_read(self, addr, count=2):
        return self.hw.i2c_read(self.i2c_addr, addr, count)

    def i2c_write(self, addr, data):
        return self.hw.i2c_write(self.i2c_addr, addr, data)

    def do_compensation(self, raw_frame, add_ambient_temperature=False):
        """
        Calculates the temperatures for each pixel
        :param raw_frame: the raw frame
        :param add_ambient_temperature: flag to add ambient temperature at the end of the frame array.
        :return: the calculated frame as a one dimensional array
        """
        if self.hw.sensor_type == 0:
            np = 32 * 24
        else:
            np = 16 * 12
        info_data = raw_frame[np:]

        if self.hw.sensor_type == 1:
            # ToDo: get VDD_pix out of info_data
            info_data[42], status = self.i2c_read(0x05AA, 2)
            if 0 != status:
                raise I2CAcknowledgeError("Error during read of Control register 1")
            info_data[42] = (struct.unpack(">H", info_data[42])[0] - 65536) if (struct.unpack(">H", info_data[42])[0] > 32767) else (struct.unpack(">H", info_data[42])[0])  # TODO find endian

            # ToDo: get Ta_PTAT out of info_data
            info_data[32], status = self.i2c_read(0x05A0, 2)
            if 0 != status:
                raise I2CAcknowledgeError("Error during read of Control register 1")
            info_data[32] = (struct.unpack(">H", info_data[32])[0] - 65536) if (struct.unpack(">H", info_data[32])[0] > 32767) else (struct.unpack(">H", info_data[32])[0])  # TODO find endian

            # ToDo: get Ta_VBE out of info_data
            info_data[0], status = self.i2c_read(0x0580, 2)
            if 0 != status:
                raise I2CAcknowledgeError("Error during read of Control register 1")
            info_data[0] = (struct.unpack(">H", info_data[0])[0] - 65536) if (struct.unpack(">H", info_data[0])[0] > 32767) else (struct.unpack(">H", info_data[0])[0])  # TODO find endian

            # ToDo: get GAIN_RAM out of info_data
            info_data[10], status = self.i2c_read(0x058A, 2)
            if 0 != status:
                raise I2CAcknowledgeError("Error during read of Control register 1")
            info_data[10] = (struct.unpack(">H", info_data[10])[0] - 65536) if (struct.unpack(">H", info_data[10])[0] > 32767) else (struct.unpack(">H", info_data[10])[0])  # TODO find endian

            # ToDo: get tgcValue out of info_data
            info_data[8], status = self.i2c_read(0x0588, 2)
            if 0 != status:
                raise I2CAcknowledgeError("Error during read of Control register 1")
            info_data[8] = (struct.unpack(">H", info_data[8])[0] - 65536) if (struct.unpack(">H", info_data[8])[0] > 32767) else (struct.unpack(">H", info_data[8])[0])  # TODO find endian


        # Calculation of actual Vdd [V] by MLX9064x
        if fabs(self.calc_params.Kv_Vdd) < 1e-6:
            raise ValueError("Kv_Vdd is too small")

        vdd_meas = self.calc_params.Vdd_V0 + (info_data[42] - self.calc_params.Vdd_25) / self.calc_params.Kv_Vdd

        # Calculation of ambient temperature
        ptat_sc = info_data[32]

        d = (ptat_sc * self.calc_params.alpha_ptat + info_data[0])
        if fabs(d) < 1e-6:
            raise ValueError("Can't calculate VPTAT_virt")

        VPTAT_virt = ptat_sc / d * (1 << 18)

        if fabs(self.calc_params.Kt_PTAT) < 1e-6:
            raise ValueError("Kt_PTAT is too small")

        d2 = (1 + (vdd_meas - self.calc_params.Vdd_V0) * self.calc_params.Kv_PTAT)
        if fabs(d2) < 1e-6:
            raise ValueError("Kv_PTAT is not correct")

        Tamb = (VPTAT_virt / d2 - self.calc_params.VPTAT_25) / self.calc_params.Kt_PTAT + 25
        info_data[2] = round((Tamb - Mlx9064x.MIN_TEMP_DEGC) * Mlx9064x.LSB_DEGC)

        lControl1 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeControl1)

        tidx = 0
        if self.calc_params.version >= 2:
            minTdiff = 9999.0
            minTidx = 0
            for tidx in range(TCalcParams.MAX_CAL_RANGES):
                if self.calc_params.Ta_min[tidx] <= Tamb <= self.calc_params.Ta_max[tidx]:
                    break
                # If Ta doesn't fall in any region? => Use the closest reg
                tdiff = min(fabs(Tamb - self.calc_params.Ta_min[tidx]),
                            fabs(Tamb - self.calc_params.Ta_max[tidx]))
                if tdiff < minTdiff:
                    minTdiff = tdiff
                    minTidx = tidx

            if tidx >= TCalcParams.MAX_CAL_RANGES:
                tidx = minTidx

        dDeltaTa = Tamb - self.calc_params.Ta0[tidx]
        dDeltaV = vdd_meas - self.calc_params.Vdd_V0

        if info_data[10] == 0:
            raise ValueError("Can't do gain drift compensation")
        dGainComp = self.calc_params.GainMeas_25_3v2 / info_data[10]

        # Compensate cyclops
        pCyclopIdx0 = [8, 9]
        pCyclopIdx1 = [0x28, 0x29]
        # int page = m_lDaqFrameIdx & 1;
        arrdCyclops = [0 for i in range(TCalcParams.NUM_PAGES)]  # accumulates all cyclops
        dKsTa = 1.0
        arrdAlphaCyclops = [0 for i in range(TCalcParams.NUM_PAGES)]
        for page in range(TCalcParams.NUM_PAGES):
            arrdCyclops[page] = 0
            arrdAlphaCyclops[page] = 0

        if self.calc_params.version >= 2:
            dKsTa = 1 + self.calc_params.KsTa * (Tamb - self.calc_params.Ta_0_Alpha)
            if fabs(dKsTa) < 1e-12:
                raise ValueError("Calculated KsTa is zero")
            for page in range(TCalcParams.NUM_PAGES):
                pCyclopIdx = pCyclopIdx1 if page else pCyclopIdx0
                for i in range(TCalcParams.NUM_TGC):
                    if fabs(self.calc_params.TGC[i]) > 1e-12:
                        tgcValue = info_data[pCyclopIdx[i]]
                        if (self.m_lDaqFrameIdx & (~1)) == 0 and self.m_lFilterTgcDepth > 1:
                            self.m_arrLastTgc[i][page] = tgcValue
                        elif self.m_lFilterTgcDepth > 1:
                            tgcValue = (self.m_arrLastTgc[i][page] * (
                            self.m_lFilterTgcDepth - 1) + tgcValue) // self.m_lFilterTgcDepth
                            self.m_arrLastTgc[i][page] = tgcValue
                            info_data[pCyclopIdx[i]] = round(
                                tgcValue)  # Allow logging of filtered TGC

                        # 1. Gain drift compensation
                        Pix_GainComp = tgcValue * dGainComp

                        # 2. Pixel offset compensation
                        Pix_os = self.calc_params.Pix_os_ref_TGC[tidx][page][i] * \
                        (1 + self.calc_params.Kta_TGC[tidx][page][i] * dDeltaTa) * \
                        (1 + self.calc_params.Kv_TGC[tidx][page][i] * dDeltaV)

                        # 3. calculating offset free IR data
                        arrdCyclops[page] += (Pix_GainComp - Pix_os) * self.calc_params.TGC[i]
                        arrdAlphaCyclops[page] += self.calc_params.alpha_TGC[page][i] * \
                                                  self.calc_params.TGC[i]

        dKsTa *= self.m_fEmissivity
        if fabs(dKsTa) < 1e-12:
            raise ValueError("Calculated KsTa is zero")

        dTaPow4 = pow(Tamb - Mlx9064x.MIN_TEMP_DEGC, 4)

        # resulting frame without service data
        if self.hw.sensor_type == 0:
            result_frame = [0] * (32 * 24)
        else:
            result_frame = [0] * (16 * 12)

        for i in range(np):
            if lControl1 & (1 << 12):
                page = (i & 1) ^ ((i // 32) & 1)  # Chess pattern mode
            else:
                page = (i // 32) % 2  # Interlaced  mode
            idxGlobal = i

            # 1. Gain drift compensation
            Pix_GainComp = raw_frame[i] * dGainComp

            # 2. Pixel offset compensation
            Pix_os_ref = 0
            Kta = 0
            Kv = 0
            if self.hw.sensor_type == 0:
                Pix_os_ref = self.calc_params.Pix_os_ref[tidx][idxGlobal]
                Kta = self.calc_params.Kta[tidx][idxGlobal]
                Kv = self.calc_params.Kv[tidx][idxGlobal]
            else:
                if page:
                    Pix_os_ref = self.calc_params.Pix_os_ref_SP1[tidx][idxGlobal]
                    Kta = self.calc_params.Kta[tidx][idxGlobal]
                    Kv = self.calc_params.Kv[tidx][idxGlobal]
                else:
                    Pix_os_ref = self.calc_params.Pix_os_ref_SP0[tidx][idxGlobal]
                    Kta = self.calc_params.Kta[tidx][idxGlobal]
                    Kv = self.calc_params.Kv[tidx][idxGlobal]

            Pix_os = Pix_os_ref * (1 + Kta * dDeltaTa) * (1 + Kv * dDeltaV)

            # 3. calculating offset free IR data
            Pix_comp = Pix_GainComp - Pix_os

            # Calculate object temperature
            alpha = self.calc_params.alpha[idxGlobal] - arrdAlphaCyclops[page]
            if fabs(alpha) < 1e-12:
                To = Mlx9064x.MIN_TEMP_DEGC
            else:
                # pass1
                d = (Pix_comp - arrdCyclops[page]) / dKsTa / alpha + dTaPow4
                if d < 0.0:
                    To = Mlx9064x.MIN_TEMP_DEGC
                else:
                    # pass2
                    To1 = pow(d, 0.25) + Mlx9064x.MIN_TEMP_DEGC
                    if self.calc_params.version >= 2:
                        dKsTo = 1 + self.calc_params.KsTo * (To1 - self.calc_params.To_0_Alpha)
                        d = (Pix_comp - arrdCyclops[page]) / dKsTa / dKsTo / alpha + dTaPow4
                        if d < 0.0:
                            To = Mlx9064x.MIN_TEMP_DEGC
                        else:
                            To = pow(d, 0.25) + Mlx9064x.MIN_TEMP_DEGC
                    else:
                        To = To1

            result_frame[i] = To
        if add_ambient_temperature:
            result_frame.append(Tamb)
        return result_frame

    def do_handle_bad_pixels(self, compensated_frame):
        result = compensated_frame
        if self.hw.sensor_type == 0:  # 90640
            for pixel_i in self.eeprom.bad_pixels:
                col = pixel_i & 0x1F
                row = pixel_i >> 5
                if pixel_i == 0 << 5 + 0:  # upper left corner
                    result[pixel_i] = (result[0 << 5 + 1] + result[1 << 5 + 0] + result[1 << 5 + 1]) / 3
                    continue
                if pixel_i == 0 << 5 + 31:  # upper right corner
                    result[pixel_i] = (result[0 << 5 + 30] + result[1 << 5 + 31] + result[1 << 5 + 30]) / 3
                    continue
                if pixel_i == 23 << 5 + 0:  # lower left corner
                    result[pixel_i] = (result[23 << 5 + 1] + result[22 << 5 + 0] + result[22 << 5 + 1]) / 3
                    continue
                if pixel_i == 23 << 5 + 31:  # lower right corner
                    result[pixel_i] = (result[23 << 5 + 30] + result[22 << 5 + 31] + result[22 << 5 + 30]) / 3
                    continue

                if col == 0:  # first col
                    result[pixel_i] = (result[(row - 1) << 5 + col] + result[(row + 1) << 5 + col] + result[
                        row << 5 + col + 1]) / 3
                    continue
                if col == 31:  # last col
                    result[pixel_i] = (result[(row - 1) << 5 + col] + result[(row + 1) << 5 + col] + result[
                        row << 5 + col - 1]) / 3
                    continue

                if row == 0:  # first row
                    result[pixel_i] = (result[row << 5 + col - 1] + result[row << 5 + col + 1] + result[
                        (row + 1) << 5 + col]) / 3
                    continue
                if row == 23:  # last row
                    result[pixel_i] = (result[row << 5 + col - 1] + result[row << 5 + col + 1] + result[
                        (row - 1) << 5 + col]) / 3
                    continue
                # pixel not on border
                result[pixel_i] = (result[row << 5 + col - 1] + result[row << 5 + col + 1] + result[
                    (row - 1) << 5 + col] + result[(row + 1) << 5 + col]) / 4

        if self.hw.sensor_type == 1:  # 90641
            for pixel_i in self.eeprom.bad_pixels:
                col = pixel_i & 0x0F
                row = pixel_i >> 4
                if pixel_i == 0 << 4 + 0:  # upper left corner
                    result[pixel_i] = (result[0 << 4 + 1] + result[1 << 4 + 0] + result[1 << 4 + 1]) / 3
                    continue
                if pixel_i == 0 << 4 + 15:  # upper right corner
                    result[pixel_i] = (result[0 << 4 + 14] + result[1 << 4 + 15] + result[1 << 4 + 14]) / 3
                    continue
                if pixel_i == 11 << 4 + 0:  # lower left corner
                    result[pixel_i] = (result[11 << 4 + 1] + result[10 << 4 + 0] + result[10 << 4 + 1]) / 3
                    continue
                if pixel_i == 11 << 4 + 15:  # lower right corner
                    result[pixel_i] = (result[11 << 4 + 14] + result[10 << 4 + 15] + result[10 << 4 + 14]) / 3
                    continue

                if col == 0:  # first col
                    result[pixel_i] = (result[(row - 1) << 4 + col] + result[(row + 1) << 4 + col] + result[
                        row << 4 + col + 1]) / 3
                    continue
                if col == 15:  # last col
                    result[pixel_i] = (result[(row - 1) << 4 + col] + result[(row + 1) << 4 + col] + result[
                        row << 4 + col - 1]) / 3
                    continue

                if row == 0:  # first row
                    result[pixel_i] = (result[row << 4 + col - 1] + result[row << 4 + col + 1] + result[
                        (row + 1) << 4 + col]) / 3
                    continue
                if row == 11:  # last row
                    result[pixel_i] = (result[row << 4 + col - 1] + result[row << 4 + col + 1] + result[
                        (row - 1) << 4 + col]) / 3
                    continue
                # pixel not on border
                result[pixel_i] = (result[row << 4 + col - 1] + result[row << 4 + col + 1] + result[
                    (row - 1) << 4 + col] + result[(row + 1) << 4 + col]) / 4

        return result

    @property
    def emissivity(self):
        return self.m_fEmissivity

    @emissivity.setter
    def emissivity(self, emissivity):
        self.m_fEmissivity = emissivity

    def calculate_parameters(self):
        """
        Calculate the necessary parameters from the eeprom. They are needed for the calculation of temperatures.
        :return: nothing
        """
        if self.hw.sensor_type == 0:
            self.calc_params.version = 5

            self.calc_params.Id0 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeID1)
            self.calc_params.Id1 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeID2)
            self.calc_params.Id2 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeID3)

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeVdd_25)
            self.calc_params.Vdd_25 = (l - 256) * (1 << 5) - (1 << 13)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeK_Vdd)
            self.calc_params.Kv_Vdd = c_int8(l).value * (1 << 5)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeRes_control)
            self.calc_params.Res_scale = (1 << l)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePTAT_25)
            self.calc_params.VPTAT_25 = c_int16(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_PTAT)
            self.calc_params.Kv_PTAT = ((l - 64) if (l > 31) else l) / (1 << 12)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKt_PTAT)
            self.calc_params.Kt_PTAT = ((l - 1024) if (l > 511) else l) / (1 << 3)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_PTAT)
            self.calc_params.alpha_ptat = l / 4 + 8
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeGAIN)
            self.calc_params.GainMeas_25_3v2 = c_int16(l).value

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePix_os_average)
            Pix_os_average = c_int16(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_Occ_rem)
            Scale_occ_rem = 1 << l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_Occ_col)
            Scale_occ_col = 1 << l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_Occ_row)
            Scale_occ_row = 1 << l
            OccRow = [0] * 24
            OccCol = [0] * 32
            for r in range(24):
                l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeOCC_row, r)
                OccRow[r] = ((l - 16) if (l > 7) else l) * Scale_occ_row

            for c in range(32):
                l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeOCC_column, c)
                OccCol[c] = ((l - 16) if (l > 7) else l) * Scale_occ_col

            for r in range(24):
                for c in range(32):
                    idx = r * 32 + c
                    l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePixel_Offset, idx)
                    self.calc_params.Pix_os_ref[0][idx] = Pix_os_average + OccRow[r] + OccCol[c] + \
                                                          ((l - 64) if (l > 31) else l) * Scale_occ_rem
                    for t in range(1, TCalcParams.MAX_CAL_RANGES):
                        self.calc_params.Pix_os_ref[t][idx] = self.calc_params.Pix_os_ref[0][idx]

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_scale)
            Alpha_scale = (1 << l) * (1 << 30)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePix_sens_average)
            Pix_sens_average = c_int16(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_Acc_rem)
            Scale_Acc_rem = 1 << l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_Acc_col)
            Scale_Acc_col = 1 << l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_Acc_row)
            Scale_Acc_row = 1 << l
            AccRow = [0] * 24
            AccCol = [0] * 32
            for r in range(24):
                l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeACC_row, r)
                AccRow[r] = ((l - 16) if (l > 7) else l) * Scale_Acc_row

            for c in range(32):
                l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeACC_column, c)
                AccCol[c] = ((l - 16) if (l > 7) else l) * Scale_Acc_col

            for r in range(24):
                for c in range(32):
                    idx = r * 32 + c
                    l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePixel_Alpha, idx)
                    self.calc_params.alpha[idx] = (Pix_sens_average + AccRow[r] + AccCol[c] +
                                                   ((l - 64) if (l > 31) else l) * Scale_Acc_rem) / Alpha_scale

            Kta = [[0, 0], [0, 0]]
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_scale1)
            Kta_scale1 = 1 << (l + 8)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_scale2)
            Kta_scale2 = 1 << l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_Avg_RO_CO)
            Kta[0][0] = c_int8(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_Avg_RO_CE)
            Kta[0][1] = c_int8(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_Avg_RE_CO)
            Kta[1][0] = c_int8(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_Avg_RE_CE)
            Kta[1][1] = c_int8(l).value
            for r in range(24):
                for c in range(32):
                    idx = r * 32 + c
                    l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePixel_Kta, idx)
                    self.calc_params.Kta[0][idx] = (((l - 8) if (l > 3) else l) * Kta_scale2 + Kta[r % 2][c % 2]) / Kta_scale1
                    for t in range(1, TCalcParams.MAX_CAL_RANGES):
                        self.calc_params.Kta[t][idx] = self.calc_params.Kta[0][idx]

            Kv = [[0, 0], [0, 0]]
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_scale)
            Kv_scale = 1 << l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_Avg_RO_CO)
            Kv[0][0] = ((l - 16) if (l > 7) else l) / Kv_scale
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_Avg_RO_CE)
            Kv[0][1] = ((l - 16) if (l > 7) else l) / Kv_scale
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_Avg_RE_CO)
            Kv[1][0] = ((l - 16) if (l > 7) else l) / Kv_scale
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_Avg_RE_CE)
            Kv[1][1] = ((l - 16) if (l > 7) else l) / Kv_scale
            for r in range(24):
                for c in range(32):
                    idx = r * 32 + c
                    self.calc_params.Kv[0][idx] = Kv[r % 2][c % 2]
                    for t in range(1, TCalcParams.MAX_CAL_RANGES):
                        self.calc_params.Kv[t][idx] = self.calc_params.Kv[0][idx]

            # as of v.2
            # self.calc_params.Vdd_V0 = 3.3;             # actual value doesn't affect the results
            self.calc_params.Ta_min[1] = -200.0
            self.calc_params.Ta_max[1] = 0.0
            self.calc_params.Ta0[1] = 25.0
            self.calc_params.Ta_min[0] = -200.0  # Fixed only R2;
            self.calc_params.Ta_max[0] = 1000.0
            self.calc_params.Ta0[0] = 25.0
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeTemp_Step)
            Temp_Step = l * 5  # TODO : (0-3) or (1-4)?
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeCT1)
            ct1 = l * Temp_Step
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeCT2)
            ct2 = l * Temp_Step
            self.calc_params.Ta_min[2] = 60.0
            self.calc_params.Ta_max[2] = ct1
            self.calc_params.Ta0[2] = 25.0
            self.calc_params.Ta_min[3] = ct1
            self.calc_params.Ta_max[3] = ct2
            self.calc_params.Ta0[3] = 25.0

            # TGC[0] is not used
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_scale)
            Alpha_scale = (1 << l) * (1 << 27)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeTGC)
            self.calc_params.TGC[1] = c_int8(l).value / (1 << 5)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_CP_P0)
            self.calc_params.alpha_TGC[0][1] = l / Alpha_scale
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_CP_P1_P0)
            self.calc_params.alpha_TGC[1][1] = self.calc_params.alpha_TGC[0][1] * \
                                               (1.0 + ((l - 64) if (l > 31) else l) / (1 << 7))
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeOffset_CP_P0)
            self.calc_params.Pix_os_ref_TGC[0][0][1] = c_double((l - 1024) if (l > 511) else l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeOffset_CP_P1_P0)
            self.calc_params.Pix_os_ref_TGC[0][1][1] = c_double((l - 64) if (l > 31) else l).value + \
                                                       self.calc_params.Pix_os_ref_TGC[0][0][1]
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_CP)
            self.calc_params.Kta_TGC[0][0][1] = c_int8(l).value / Kta_scale1
            self.calc_params.Kta_TGC[0][1][1] = self.calc_params.Kta_TGC[0][0][1]
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_CP)
            self.calc_params.Kv_TGC[0][0][1] = c_int8(l).value / Kv_scale
            self.calc_params.Kv_TGC[0][1][1] = self.calc_params.Kv_TGC[0][0][1]
            for page in range(TCalcParams.NUM_PAGES):
                for t in range(1, TCalcParams.MAX_CAL_RANGES):
                    self.calc_params.Pix_os_ref_TGC[t][page][1] = self.calc_params.Pix_os_ref_TGC[0][page][1]
                    self.calc_params.Kta_TGC[t][page][1] = self.calc_params.Kta_TGC[0][page][1]
                    self.calc_params.Kv_TGC[t][page][1] = self.calc_params.Kv_TGC[0][page][1]

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKsTa)
            self.calc_params.KsTa = c_int8(l).value / (1 << 13)
            # Ta_0_Alpha = 25;
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_KsTo)
            ScaleKsTo = 1 << (l + 8)
            # Only R2 (extended -200-1000)degC
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKsTo_R2)
            self.calc_params.KsTo = c_int8(l).value / ScaleKsTo
            # To_0_Alpha;            // default 0.0
        else:
            self.calc_params.version = 5

            self.calc_params.Id0 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeID1)
            self.calc_params.Id1 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeID2)
            self.calc_params.Id2 = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeID3)

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeVdd_25)
            self.calc_params.Vdd_25 = ((l - 2048) if (l>1023) else l) * 2**5
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeK_Vdd)
            self.calc_params.Kv_Vdd = ((l - 2048) if (l>1023) else l) * 2**5
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeCalib_res_cont)
            self.calc_params.Res_scale = l
            l = 32 * self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodePTAT_25), 0) + self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodePTAT_25), 1)
            self.calc_params.VPTAT_25 = c_int16(l).value
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_PTAT)
            self.calc_params.Kv_PTAT = ((l - 2048) if (l > 1023) else l) / 2**12
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKt_PTAT)
            self.calc_params.Kt_PTAT = ((l - 2048) if (l > 1023) else l) / 2**3
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_PTAT)
            self.calc_params.alpha_ptat = l / 2**7
            l = 32 * self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeGAIN, 0) + self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeGAIN, 1)
            self.calc_params.GainMeas_25_3v2 = c_int16(l).value

            l = 32 * self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodePix_os_average), 0) + self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodePix_os_average), 1)
            offset_average = ((l - 65536) if (l>32767) else l)
            offset_scale = self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodeScale_occ_os))
            for r in range(12):
                for c in range(16):
                    idx = r * 16 + c
                    l = self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodePixel_Offset), idx)
                    self.calc_params.Pix_os_ref_SP0[0][idx] = offset_average + ((l - 2048) if (l > 1023) else l) * 2 ** offset_scale
                    l = self.eeprom.get_parameter_code((ParameterCodesEEPROM.CodePixel_os), idx)
                    self.calc_params.Pix_os_ref_SP1[0][idx] = offset_average + ((l - 2048) if (l > 1023) else l) * 2 ** offset_scale
                    for t in range(1, TCalcParams.MAX_CAL_RANGES):
                        self.calc_params.Pix_os_ref_SP0[t][idx] = self.calc_params.Pix_os_ref_SP0[0][idx]
                        self.calc_params.Pix_os_ref_SP1[t][idx] = self.calc_params.Pix_os_ref_SP1[0][idx]

            for r in range(12):
                for c in range(16):
                    idx = r * 16 + c
                    pixel_row = int(((16*(r-1)+c)-1) / 32)
                    alpha_reference = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeRow_max, pixel_row) / (2**(self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeScale_row, pixel_row)+20))
                    l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePixel_Sensitivity, idx)
                    self.calc_params.alpha[idx] = l / (2**11-1) * alpha_reference

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_avg)
            Kta = ((l - 2048) if (l > 1023) else l)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_scale_1)
            Kta_scale1 = 2**l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_scale_2)
            Kta_scale2 = 2**l

            for r in range(12):
                for c in range(16):
                    idx = r * 16 + c
                    l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePixel_Kta, idx)
                    self.calc_params.Kta[0][idx] = (((l - 64) if (l > 31) else l) * Kta_scale2 + Kta) / Kta_scale1
                    for t in range(1, TCalcParams.MAX_CAL_RANGES):
                        self.calc_params.Kta[t][idx] = self.calc_params.Kta[0][idx]

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_avg)
            Kv = ((l - 2048) if (l > 1023) else l)
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_scale_1)
            Kv_scale1 = 2**l
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_scale_2)
            Kv_scale2 = 2**l

            for r in range(12):
                for c in range(16):
                    idx = r * 16 + c
                    l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodePixel_Kv, idx)
                    self.calc_params.Kv[0][idx] = (((l - 64) if (l > 31) else l) * Kv_scale2 + Kv) / Kv_scale1
                    for t in range(1, TCalcParams.MAX_CAL_RANGES):
                        self.calc_params.Kv[t][idx] = self.calc_params.Kv[0][idx]
            # as of v.2
            # self.calc_params.Vdd_V0 = 3.3;             # actual value doesn't affect the results
            self.calc_params.Ta_min[0] = -40.0  # Fixed only R2;
            self.calc_params.Ta_max[0] = 20.0
            self.calc_params.Ta0[0] = 25.0
            self.calc_params.Ta_min[1] = -20.0
            self.calc_params.Ta_max[1] = 0.0
            self.calc_params.Ta0[1] = 25.0
            self.calc_params.Ta_min[2] = 0.0
            self.calc_params.Ta_max[2] = 80.0
            self.calc_params.Ta0[2] = 25.0
            self.calc_params.Ta_min[3] = 80.0
            self.calc_params.Ta_max[3] = 120.0
            self.calc_params.Ta0[3] = 25.0

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeTGC)
            self.calc_params.TGC[1] = 0#l / 2**6

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_CP_scale)
            Alpha_scale = l / 2**38
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeAlpha_CP)
            self.calc_params.alpha_TGC[0][1] = l / Alpha_scale

            self.calc_params.Kta_TGC[0][0][1] = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_CP) / (2**(self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKta_CP_scale)))
            self.calc_params.Kta_TGC[0][1][1] = self.calc_params.Kta_TGC[0][0][1]

            self.calc_params.Kv_TGC[0][0][1] = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_CP) / (2**(self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKv_CP_scale)))
            self.calc_params.Kv_TGC[0][1][1] = self.calc_params.Kv_TGC[0][0][1]

            for page in range(TCalcParams.NUM_PAGES):
                for t in range(1, TCalcParams.MAX_CAL_RANGES):
                    self.calc_params.Kta_TGC[t][page][1] = self.calc_params.Kta_TGC[0][page][1]
                    self.calc_params.Kv_TGC[t][page][1] = self.calc_params.Kv_TGC[0][page][1]

            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKsTa)
            self.calc_params.KsTa = ((l - 2048) if (l > 1023) else l)  / 2**15
            # Ta_0_Alpha = 25;
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKsTo_scale)
            ScaleKsTo = 2 ** l
            # Only R2 (extended -200-1000)degC
            l = self.eeprom.get_parameter_code(ParameterCodesEEPROM.CodeKsTo_R2)
            self.calc_params.KsTo =  ((l - 2048) if (l > 1023) else l) / ScaleKsTo
            # To_0_Alpha;            // default 0.0

    @staticmethod
    def fake_read_frame(name: str):
        """
        Verify the calculation using a log from MlxCIRT
        :param str name: name and path of the csv log generated from the demo
        :return: (processed, raw) a frame for each call
        """
        with open(name) as f:
            log = f.readlines()

        log = log[2:]

        for l in log:
            l = l.split('||')
            temps = [float(i) for i in l[0].split('|')[2:]]
            raw = [float(i) for i in l[2].split('|')]
            yield temps, raw


class Mlx9064xFilters:
    def deinterlace_filter(ir_image, subpage):
        if subpage == 1:
            chess = 0
            for x in range(0, 24):
                for y in range(chess, 32, 2):
                    if (x == 0):
                        if (y == 0):
                            a = median([ir_image[x+1][y], ir_image[x][y+1], ir_image[x][y]])
                        else:
                            a = median([ir_image[x][y-1], ir_image[x+1][y], ir_image[x][y+1], ir_image[x][y]])
                    elif (x == 23):
                        if (y == 31):
                            a = median([ir_image[x-1][y], ir_image[x][y-1], ir_image[x][y]])
                        else:
                            a = median([ir_image[x][y-1], ir_image[x-1][y], ir_image[x][y+1], ir_image[x][y]])
                    elif ((y == 0) and (x%2 == 0)):
                        a = median([ir_image[x-1][y], ir_image[x+1][y], ir_image[x][y+1], ir_image[x][y]])
                    elif ((y == 31) and (x%2 == 1)):
                        a = median([ir_image[x][y-1], ir_image[x-1][y], ir_image[x+1][y], ir_image[x][y]])
                    else:
                        a = median([ir_image[x-1][y], ir_image[x+1][y], ir_image[x][y-1], ir_image[x][y+1], ir_image[x][y], ir_image[x][y]])
                    if abs(a-ir_image[x][y]) > 0.7:
                        ir_image[x][y] = a
                if chess == 0:
                    chess = 1
                else:
                    chess = 0
        else:
            chess = 1
            for x in range(0, 24):
                for y in range(chess, 32, 2):
                    if x == 0:
                        if y == 31:
                            a = median([ir_image[x][y-1], ir_image[x+1][y], ir_image[x][y]])
                        else:
                            a = median([ir_image[x][y-1], ir_image[x+1][y], ir_image[x][y+1], ir_image[x][y]])
                    elif x == 23:
                        if y == 0:
                            a = median([ir_image[x-1][y], ir_image[x][y+1], ir_image[x][y]])
                        else:
                            a = median([ir_image[x][y-1], ir_image[x-1][y], ir_image[x][y+1], ir_image[x][y]])
                    elif y == 0 and x%2 == 1:
                        a = median([ir_image[x-1][y], ir_image[x+1][y], ir_image[x][y+1], ir_image[x][y]])
                    elif y == 31 and x%2 == 0:
                        a = median([ir_image[x][y-1], ir_image[x-1][y], ir_image[x+1][y], ir_image[x][y]])
                    else:
                        a = median([ir_image[x-1][y], ir_image[x+1][y], ir_image[x][y-1], ir_image[x][y+1], ir_image[x][y], ir_image[x][y]])
                    if abs(a-ir_image[x][y]) > 0.7:
                        ir_image[x][y] = a
                if chess ==1:
                    chess = 0
                else:
                    chess = 1
        return ir_image

    def iir_filter(ir_image, iir_ir_image, cnt, depth=8, threshold=2.5):
        for x in range(0, 24):
            for y in range(0, 32):
                if abs(iir_ir_image[x][y] - ir_image[x][y]) >= threshold:
                    iir_ir_image[x][y] = ir_image[x][y]
                    cnt[x][y] = 1
                else:
                    if cnt[x][y] < depth:
                        iir_ir_image[x][y] = (iir_ir_image[x][y] * cnt[x][y] + iir_ir_image[x][y]) / (cnt[x][y] + 1)
                        cnt[x][y] = cnt[x][y] + 1
                    else:
                        iir_ir_image[x][y] = (ir_image[x][y] + iir_ir_image[x][y] * (depth - 1)) / depth
        return iir_ir_image, cnt


class ParameterCodesEEPROM(enum.Enum):
    CodeOscTrim = 1
    CodeAnalogTrim = 2
    CodeConfiguration = 3
    CodeI2CAddr = 4
    CodeCropPageAddr = 5
    CodeCropCellAddr = 6
    CodeControl1 = 7
    CodeControl2 = 8
    CodeI2CConf = 9
    CodeID1 = 10
    CodeID2 = 11
    CodeID3 = 12
    CodeDeviceOptions = 13
    CodeAnalogTrim2 = 14
    CodeScale_Occ_rem = 15
    CodeScale_Occ_col = 16
    CodeScale_Occ_row = 17
    CodeAlpha_PTAT = 18
    CodePix_os_average = 19
    CodeOCC_row = 20
    CodeOCC_column = 21
    CodeScale_Acc_rem = 22
    CodeScale_Acc_col = 23
    CodeScale_Acc_row = 24
    CodeAlpha_scale = 25
    CodePix_sens_average = 26
    CodeACC_row = 27
    CodeACC_column = 28
    CodeGAIN = 29
    CodePTAT_25 = 30
    CodeKt_PTAT = 31
    CodeKv_PTAT = 32
    CodeVdd_25 = 33
    CodeK_Vdd = 34
    CodeKv_Avg_RE_CE = 35
    CodeKv_Avg_RO_CE = 36
    CodeKv_Avg_RE_CO = 37
    CodeKv_Avg_RO_CO = 38
    CodeKta_Avg_RE_CO = 39
    CodeKta_Avg_RO_CO = 40
    CodeKta_Avg_RE_CE = 41
    CodeKta_Avg_RO_CE = 42
    CodeKta_scale2 = 43
    CodeKta_scale1 = 44
    CodeKv_scale = 45
    CodeRes_control = 46
    CodeAlpha_CP_P0 = 47
    CodeAlpha_CP_P1_P0 = 48
    CodeOffset_CP_P0 = 49
    CodeOffset_CP_P1_P0 = 50
    CodeKta_CP = 51
    CodeKv_CP = 52
    CodeTGC = 53
    CodeKsTa = 54
    CodeKsTo_R1 = 55
    CodeKsTo_R2 = 56
    CodeKsTo_R3 = 57
    CodeKsTo_R4 = 58
    CodeScale_KsTo = 59
    CodeCT1 = 60
    CodeCT2 = 61
    CodeTemp_Step = 62
    CodePixel_Kta = 63
    CodePixel_Alpha = 64
    CodePixel_Offset = 65
    CodeScale_occ_os = 66
    CodePix_os_avg = 67
    CodeKta_avg = 68
    CodeKta_scale_2 = 69
    CodeKta_scale_1 = 70
    CodeKv_avg = 71
    CodeKv_scale_2 = 72
    CodeKv_scale_1 = 73
    CodeScale_row = 74
    CodeRow_max = 75
    CodeEmissivity = 76
    CodeAlpha_CP = 77
    CodeAlpha_CP_scale = 78
    CodeOffset_CP = 79
    CodeKta_CP_scale = 80
    CodeKv_CP_scale = 81
    CodeCalib_res_cont = 82
    CodeKsTo_scale = 83
    CodeKsTo = 84
    CodePixel_os = 85
    CodePixel_Sensitivity = 86
    CodePixel_Kv = 87


class TCalcParams:
    MAX_IR_COLS = 32
    MAX_IR_ROWS = 24
    MAX_IR_PIXELS = MAX_IR_COLS * MAX_IR_ROWS
    MAX_IR_COLS_90641 = 16
    MAX_IR_ROWS_90641 = 12
    MAX_IR_PIXELS_90641 = MAX_IR_COLS_90641 * MAX_IR_ROWS_90641
    MAX_CAL_RANGES = 4
    NUM_TGC = 2
    NUM_PAGES = 2

    def __init__(self):
        self.version = 0
        self.Id0 = 0
        self.Id1 = 0
        self.Id2 = 0                         # Two 32-bit numbers for chip ID
        self.Vdd_25 = 0                      # LSB16,Value of VDD measurement at 25 degC and 3.2V supply
        self.Kv_Vdd = 0                      # LSB16/V,slope of VDD measurements
        self.Res_scale = 0                   # V/V,Scaling coefficient of #resolution
        self.VPTAT_25 = 0                    # LSB18,VPTAT value for 5 degC and Vdd=3.2V
        self.Kv_PTAT = 0                     # LSB18/V,Supply dependence of VPTAT
        self.Kt_PTAT = 0                     # LSB18/degC,Slope of PTAT
        self.alpha_ptat = 0                  # V/V,Virtual reference coefficient
        self.GainMeas_25_3v2 = 0             # LSB,Gain measurement channel for 3.2V supply and 25 degC
        self.Pix_os_ref = [[0.0 for t in range(TCalcParams.MAX_IR_PIXELS)] for m in range(TCalcParams.MAX_CAL_RANGES)] # 32x24 array,LSB,32x24=768 constants for offset per pixel at Tamb=25 degC and 3.2V supply
        self.Pix_os_ref_SP0 = [[0.0 for t in range(TCalcParams.MAX_IR_PIXELS_90641)] for m in range(TCalcParams.MAX_CAL_RANGES)] # 16x12 array,LSB,16x12=192 constants for offset per pixel at Tamb=25 degC and 3.2V supply
        self.Pix_os_ref_SP1 = [[0.0 for t in range(TCalcParams.MAX_IR_PIXELS_90641)] for m in range(TCalcParams.MAX_CAL_RANGES)] # 16x12 array,LSB,16x12=192 constants for offset per pixel at Tamb=25 degC and 3.2V supply
        self.Kta = [[0.0 for t in range(TCalcParams.MAX_IR_PIXELS)] for m in range(TCalcParams.MAX_CAL_RANGES)]    # 32x24 array,LSB/degC,32x24=768 constants for offset dependence vs Tamb
        self.Kv = [[0.0 for t in range(TCalcParams.MAX_IR_PIXELS)] for m in range(TCalcParams.MAX_CAL_RANGES)]        # 32x24 array,LSB/V,32x24=768 constants for offset dependence vs supply
        self.alpha = [1.0 for t in range(TCalcParams.MAX_IR_PIXELS)]        # 32x24 array,LSB/K^4,Sensitivity of pixels

        # as of v.2
        self.Vdd_V0 = 0
        self.Ta_min = [0.0 for t in range(TCalcParams.MAX_CAL_RANGES)]
        self.Ta_max = [0.0 for t in range(TCalcParams.MAX_CAL_RANGES)]
        self.Ta0 = [0.0 for t in range(TCalcParams.MAX_CAL_RANGES)]
        self.TGC = [0.0 for t in range(TCalcParams.NUM_TGC)]
        self.Pix_os_ref_TGC = [[[0.0 for t in range(TCalcParams.NUM_TGC)] for p in range(TCalcParams.NUM_PAGES)]
                               for m in range(TCalcParams.MAX_CAL_RANGES)]
        self.Kta_TGC = [[[0.0 for t in range(TCalcParams.NUM_TGC)] for p in range(TCalcParams.NUM_PAGES)]
                        for m in range(TCalcParams.MAX_CAL_RANGES)]
        self.Kv_TGC = [[[0.0 for t in range(TCalcParams.NUM_TGC)] for p in range(TCalcParams.NUM_PAGES)]
                       for m in range(TCalcParams.MAX_CAL_RANGES)]
        self.alpha_TGC = [[0.0 for t in range(TCalcParams.NUM_TGC)] for p in range(TCalcParams.NUM_PAGES)]
        self.KsTa = 0
        self.Ta_0_Alpha = 0
        self.KsTo = 0
        self.To_0_Alpha = 0

        self.set_defaults()

    def set_defaults(self):
        self.version = 0
        self.Id0 = 0
        self.Id1 = 0
        self.Id2 = 0
        self.Vdd_25 = -19474
        self.Kv_Vdd = -4690
        self.Res_scale = 8
        self.VPTAT_25 = 10974
        self.Kv_PTAT = 0.0113
        self.Kt_PTAT = 35.74
        self.alpha_ptat = 11.2
        self.GainMeas_25_3v2 = 5471

        self.Vdd_V0 = 3.3
        self.KsTa = 0.001
        self.Ta_0_Alpha = 25.0
        self.KsTo = 0.0004
        self.To_0_Alpha = 0.0

        arrTa_min = [-40.0,  70.0, 110.0, 900]
        arrTa_max = [ 70.0, 110.0, 150.0, 800]
        arrTa0    = [ 25.0,  90.0, 130.0, 900]

        for t in range(TCalcParams.MAX_CAL_RANGES):
            self.Ta_min[t] = arrTa_min[t]
            self.Ta_max[t] = arrTa_max[t]
            self.Ta0[t] = arrTa0[t]
            for i in range(TCalcParams.MAX_IR_PIXELS):
                self.Pix_os_ref[t][i] = 0
                self.Kta[t][i] = 0
                self.Kv[t][i] = 0

        for i in range(TCalcParams.MAX_IR_PIXELS):
            self.alpha[i] = 1

        for i in range(TCalcParams.NUM_TGC):
            self.TGC[i] = 0
            for page in range(TCalcParams.NUM_PAGES):
                self.alpha_TGC[page][i] = 1
                for t in range(TCalcParams.MAX_CAL_RANGES):
                    self.Pix_os_ref_TGC[t][page][i] = 0
                    self.Kta_TGC[t][page][i] = 0
                    self.Kv_TGC[t][page][i] = 0


class Mlx90640EEPROM:
    eeprom_map = {
        ParameterCodesEEPROM.CodeOscTrim: 0,
        ParameterCodesEEPROM.CodeAnalogTrim: 1,
        ParameterCodesEEPROM.CodeConfiguration: 3,
        ParameterCodesEEPROM.CodeI2CAddr: 4,
        ParameterCodesEEPROM.CodeAnalogTrim2: None,
        ParameterCodesEEPROM.CodeCropPageAddr: 5,
        ParameterCodesEEPROM.CodeCropCellAddr: 6,
        ParameterCodesEEPROM.CodeID1: 7,
        ParameterCodesEEPROM.CodeID2: 8,
        ParameterCodesEEPROM.CodeID3: 9,
        ParameterCodesEEPROM.CodeDeviceOptions: 10,
        ParameterCodesEEPROM.CodeControl1: 0xC,
        ParameterCodesEEPROM.CodeControl2: 0xD,
        ParameterCodesEEPROM.CodeI2CConf: 0xE,
        #  ParameterCodesEEPROM.CodeEepromIdVersion: -1,
        #  ParameterCodesEEPROM.CodeArraySize: -1,
        ParameterCodesEEPROM.CodeScale_Occ_rem: [0x10, 0, 4],
        ParameterCodesEEPROM.CodeScale_Occ_col: [0x10, 4, 4],
        ParameterCodesEEPROM.CodeScale_Occ_row: [0x10, 8, 4],
        ParameterCodesEEPROM.CodeAlpha_PTAT: [0x10, 12, 4],
        ParameterCodesEEPROM.CodePix_os_average: [0x11, 0, 16],
        ParameterCodesEEPROM.CodeScale_Acc_rem: [0x20, 0, 4],
        ParameterCodesEEPROM.CodeScale_Acc_col: [0x20, 4, 4],
        ParameterCodesEEPROM.CodeScale_Acc_row: [0x20, 8, 4],
        ParameterCodesEEPROM.CodeAlpha_scale: [0x20, 12, 4],
        ParameterCodesEEPROM.CodePix_sens_average: [0x21, 0, 16],
        ParameterCodesEEPROM.CodeGAIN: [0x30, 0, 16],
        ParameterCodesEEPROM.CodePTAT_25: [0x31, 0, 16],
        ParameterCodesEEPROM.CodeKt_PTAT: [0x32, 0, 10],
        ParameterCodesEEPROM.CodeKv_PTAT: [0x32, 10, 6],
        ParameterCodesEEPROM.CodeVdd_25: [0x33, 0, 8],
        ParameterCodesEEPROM.CodeK_Vdd: [0x33, 8, 8],
        ParameterCodesEEPROM.CodeKv_CP: [0x3B, 8, 8],
        ParameterCodesEEPROM.CodeRes_control: [0x38, 12, 2],
        ParameterCodesEEPROM.CodeKsTo_R3: [0x3E, 0, 8],
        ParameterCodesEEPROM.CodeCT2: [0x3F, 8, 4],
        ParameterCodesEEPROM.CodeKv_Avg_RO_CO: [0x34, 12, 4],
        ParameterCodesEEPROM.CodeKta_scale2: [0x38, 0, 4],
        ParameterCodesEEPROM.CodeCT1: [0x3F, 4, 4],
        ParameterCodesEEPROM.CodeOffset_CP_P1_P0: [0x3A, 10, 6],
        ParameterCodesEEPROM.CodeTemp_Step: [0x3F, 12, 2],
        ParameterCodesEEPROM.CodeKsTo_R1: [0x3D, 0, 8],
        ParameterCodesEEPROM.CodeKv_scale: [0x38, 8, 4],
        ParameterCodesEEPROM.CodeKta_scale1: [0x38, 4, 4],
        ParameterCodesEEPROM.CodeKsTo_R4: [0x3E, 8, 8],
        ParameterCodesEEPROM.CodeKv_Avg_RO_CE: [0x34, 4, 4],
        ParameterCodesEEPROM.CodeOffset_CP_P0: [0x3A, 0, 10],
        ParameterCodesEEPROM.CodeKta_Avg_RO_CO: [0x36, 8, 8],
        ParameterCodesEEPROM.CodeAlpha_CP_P0: [0x39, 0, 10],
        ParameterCodesEEPROM.CodeKsTo_R2: [0x3D, 8, 8],
        ParameterCodesEEPROM.CodeKta_Avg_RE_CO: [0x36, 0, 8],
        ParameterCodesEEPROM.CodeScale_KsTo: [0x3F, 0, 4],
        ParameterCodesEEPROM.CodeKv_Avg_RE_CO: [0x34, 8, 4],
        ParameterCodesEEPROM.CodeTGC: [0x3C, 0, 8],
        ParameterCodesEEPROM.CodeKta_Avg_RE_CE: [0x37, 0, 8],
        ParameterCodesEEPROM.CodeAlpha_CP_P1_P0: [0x39, 10, 6],
        ParameterCodesEEPROM.CodeKta_Avg_RO_CE: [0x37, 8, 8],
        ParameterCodesEEPROM.CodeKsTa: [0x3C, 8, 8],
        ParameterCodesEEPROM.CodeKta_CP: [0x3B, 0, 8],
        ParameterCodesEEPROM.CodeKv_Avg_RE_CE: [0x34, 0, 4],
        ParameterCodesEEPROM.CodeOCC_row: (0, 24, lambda index: [0x12 + index // 4, (index % 4) * 4, 4]),
        ParameterCodesEEPROM.CodeOCC_column: (0, 32, lambda index: [0x18 + index // 4, (index % 4) * 4, 4]),
        ParameterCodesEEPROM.CodeACC_row: (0, 24, lambda index: [0x22 + index // 4, (index % 4) * 4, 4]),
        ParameterCodesEEPROM.CodeACC_column: (0, 32, lambda index: [0x28 + index // 4, (index % 4) * 4, 4]),
        ParameterCodesEEPROM.CodePixel_Kta: (0, 32 * 24, lambda index: [0x40 + index, 1, 3]),
        ParameterCodesEEPROM.CodePixel_Alpha: (0, 32 * 24, lambda index: [0x40 + index, 4, 6]),
        ParameterCodesEEPROM.CodePixel_Offset: (0, 32 * 24, lambda index: [0x40 + index, 10, 6])
    }

    def __init__(self, device: Mlx9064x):
        self.device = device
        self.eeprom = None
        self.eeprom_size = 0x680
        self.outlier_pixels = []
        self.broken_pixels = []
        self.bad_pixels = []

    def get_bit(self, index, lsb):
        return (self.eeprom[index] & (1 << lsb)) != 0

    def get_bits(self, index, lsb, nbits):
        return (self.eeprom[index] >> lsb) & ((1 << nbits) - 1)

    def set_bits(self, index, data, lsb, nbits):
        mask = (1 << nbits) - 1
        self.eeprom[index] = (self.eeprom[index] & ~(mask << lsb)) | ((data & mask) << lsb)

    def get_eeprom_id_version(self):
        if (self.eeprom[0x09] == 0x100C and self.eeprom[0x0A] == 0x0020) or \
                (self.eeprom[0x09] == 0 and self.eeprom[0x0A] == 0):
            return 0
        else:
            return 1

    def read_eeprom_from_device(self):
        """
        Perform consecutive reads of the EEPROM to ensure a correct read, sets self.eeprom as a list of 16 bit
        integers read in big endian
        :returns: nothing
        :raises: ValueError - error during read from chip
        """
        first_read, status = self.device.hw.i2c_read(self.device.i2c_addr, 0x2400, self.eeprom_size)
        if status != 0:
            raise ValueError("Error during initial read of eeprom")
        first_read = list(struct.unpack(">" + str(self.eeprom_size // 2) + "H", first_read))

        for m in range(2):
            consecutive_read, status = self.device.hw.i2c_read(self.device.i2c_addr, 0x2400, self.eeprom_size)
            if status != 0:
                raise ValueError("Error during consecutive read of eeprom")
            consecutive_read = list(struct.unpack(">" + str(self.eeprom_size // 2) + "H", consecutive_read))

            for i in range(self.eeprom_size // 2):
                first_read[i] |= consecutive_read[i]
        self.eeprom = first_read
        self.extract_broken_pixels()
        self.extract_outlier_pixels()
        self.get_bad_pixels()

    def get_parameter_code(self, param_id: ParameterCodesEEPROM, index=None):
        """
        Gets a named parameter from the eeprom. If it is an indexed parameter the index will be checked.
        :param ParameterCodesEEPROM param_id: the id of the parameter
        :param int index: only needed for indexed parameters
        :return: nothing
        :raises: ValueError - missing eeprom (not read from device), or the eeprom parameter is not defined
                 AttributeError - index not provided when needed
                 IndexError - index is out of predefined range for parameter
        """
        if self.eeprom is None:
            raise ValueError("EEPROM is not read from device")
        params = Mlx90640EEPROM.eeprom_map[param_id]
        if type(params) is int:
            return self.eeprom[params]
        elif type(params) is list:
            return self.get_bits(*params)
        elif type(params) is tuple:
            if index is None:
                raise AttributeError("For this parameter index can not be None")

            if params[0] <= index < params[1]:
                calculated_params = params[2](index)
                return self.get_bits(*calculated_params)
            else:
                raise IndexError("index: {} out of range ({}, {})".format(index, params[0], params[1]))
        else:
            ValueError("invalid eeprom parameter at {}".format(param_id))

    def extract_outlier_pixels(self):
        self.outlier_pixels = []
        for pixel_i in range(768):
            if self.eeprom[pixel_i + 64] & 0x0001:
                self.outlier_pixels.append(pixel_i)
        return self.outlier_pixels

    def extract_broken_pixels(self):
        self.broken_pixels = []
        for pixel_i in range(768):
            if self.eeprom[pixel_i + 64] == 0:
                self.broken_pixels.append(pixel_i)
        return self.broken_pixels

    def get_bad_pixels(self):
        self.bad_pixels = self.broken_pixels
        for pixel_i in self.outlier_pixels:
            if pixel_i not in self.bad_pixels:
                self.bad_pixels.append(pixel_i)
        return self.bad_pixels


class Mlx90641EEPROM:
    eeprom_map = {
        ParameterCodesEEPROM.CodeOscTrim: 0,
        ParameterCodesEEPROM.CodeAnalogTrim: 1,
        ParameterCodesEEPROM.CodeConfiguration: 3,
        ParameterCodesEEPROM.CodeI2CAddr: 0xF,  # (ChipV == ChipVersion90640AAA || ChipV == ChipVersion90641AAA)
        ParameterCodesEEPROM.CodeAnalogTrim2: 4,  # (ChipV == ChipVersion90640AAA || ChipV == ChipVersion90641AAA)
        ParameterCodesEEPROM.CodeCropPageAddr: 5,
        ParameterCodesEEPROM.CodeCropCellAddr: 6,
        ParameterCodesEEPROM.CodeID1: 7,
        ParameterCodesEEPROM.CodeID2: 8,
        ParameterCodesEEPROM.CodeID3: 9,
        ParameterCodesEEPROM.CodeDeviceOptions: 10,
        ParameterCodesEEPROM.CodeControl1: 0xC,
        ParameterCodesEEPROM.CodeControl2: 0xD,
        ParameterCodesEEPROM.CodeI2CConf: 0xE,
        ParameterCodesEEPROM.CodeScale_occ_os: [0x10, 6, 6],
        ParameterCodesEEPROM.CodePix_os_average: (0, 2, lambda index: [0x11 + index, 0, 11]),
        ParameterCodesEEPROM.CodeKta_avg: [0x15, 0, 11],
        ParameterCodesEEPROM.CodeKta_scale_2: [0x16, 0, 5],
        ParameterCodesEEPROM.CodeKta_scale_1: [0x16, 5, 6],
        ParameterCodesEEPROM.CodeKv_avg: [0x17, 0, 11],
        ParameterCodesEEPROM.CodeKv_scale_2: [0x18, 0, 5],
        ParameterCodesEEPROM.CodeKv_scale_1: [0x18, 5, 6],
        ParameterCodesEEPROM.CodeScale_row: (0,6, lambda index: [0x19 + index // 2, abs((index%2)-1)*5, abs((index%2)-1)+5]),
        ParameterCodesEEPROM.CodeRow_max: (0, 6, lambda index: [0x1C + index, 0, 11]),
        ParameterCodesEEPROM.CodeKsTa: [0x22, 0, 11],
        ParameterCodesEEPROM.CodeEmissivity: [0x23, 0, 11],
        ParameterCodesEEPROM.CodeGAIN: (0, 2, lambda index: [0x24 + index, 0, 11]),
        ParameterCodesEEPROM.CodeVdd_25: [0x26, 0, 11],
        ParameterCodesEEPROM.CodeK_Vdd: [0x27, 0, 11],
        ParameterCodesEEPROM.CodePTAT_25: (0, 2, lambda index: [0x28 + index, 0, 11]),
        ParameterCodesEEPROM.CodeKt_PTAT: [0x2A, 0, 11],
        ParameterCodesEEPROM.CodeKv_PTAT: [0x2B, 0, 11],
        ParameterCodesEEPROM.CodeAlpha_PTAT: [0x2C, 0, 11],
        ParameterCodesEEPROM.CodeAlpha_CP: [0x2D, 0, 11],
        ParameterCodesEEPROM.CodeAlpha_CP_scale: [0x2E, 0, 11],
        ParameterCodesEEPROM.CodeOffset_CP: (0, 2, lambda index: [0x2F + index, 0, 11]),
        ParameterCodesEEPROM.CodeKta_CP: [0x31, 0, 6],
        ParameterCodesEEPROM.CodeKta_CP_scale: [0x31, 6, 5],
        ParameterCodesEEPROM.CodeKv_CP: [0x32, 0, 6],
        ParameterCodesEEPROM.CodeKv_CP_scale: [0x32, 6, 5],
        ParameterCodesEEPROM.CodeTGC: [0x33, 0, 9],
        ParameterCodesEEPROM.CodeCalib_res_cont: [0x33, 9, 2],
        ParameterCodesEEPROM.CodeKsTo_scale: [0x34, 0, 11],
        ParameterCodesEEPROM.CodeKsTo_R1: [0x35, 0, 11],
        ParameterCodesEEPROM.CodeKsTo_R2: [0x36, 0, 11],
        ParameterCodesEEPROM.CodeKsTo_R3: [0x37, 0, 11],
        ParameterCodesEEPROM.CodeKsTo_R4: [0x39, 0, 11],
        #ParameterCodesEEPROM.CodeKsTo_R5: [0x39, 0, 11],
        #ParameterCodesEEPROM.CodeCT6: [0x3A, 0, 11],
        #ParameterCodesEEPROM.CodeKsTo_R6: [0x3B, 0, 11],
        #ParameterCodesEEPROM.CodeCT7: [0x3C, 0, 11],
        #ParameterCodesEEPROM.CodeKsTo_R7: [0x3D, 0, 11],
        #ParameterCodesEEPROM.CodeCT8: [0x3E, 0, 11],
        #ParameterCodesEEPROM.CodeKsTo_R8: [0x3F, 0, 11],
        ParameterCodesEEPROM.CodePixel_Offset: (0, 16 * 12, lambda index: [0x40 + index, 0, 11]),
        ParameterCodesEEPROM.CodePixel_Sensitivity: (0, 16 * 12, lambda index: [0x100 + index, 0, 11]),
        ParameterCodesEEPROM.CodePixel_Kv: (0, 16 * 12, lambda index: [0x1C0 + index, 0, 5]),
        ParameterCodesEEPROM.CodePixel_Kta: (0, 16 * 12, lambda index: [0x1C0 + index, 5, 6]),
        ParameterCodesEEPROM.CodePixel_os: (0, 16 * 12, lambda index: [0x280 + index, 0, 11])
    }

    def __init__(self, device: Mlx9064x):
        self.device = device
        self.eeprom = None
        self.eeprom_size = 0x680
        self.bad_pixels = []

    def get_bit(self, index, lsb):
        return (self.eeprom[index] & (1 << lsb)) != 0

    def get_bits(self, index, lsb, nbits):
        return (self.eeprom[index] >> lsb) & ((1 << nbits) - 1)

    def set_bits(self, index, data, lsb, nbits):
        mask = (1 << nbits) - 1
        self.eeprom[index] = (self.eeprom[index] & ~(mask << lsb)) | ((data & mask) << lsb)

    def get_eeprom_id_version(self):
        if (self.eeprom[0x09] == 0x100C and self.eeprom[0x0A] == 0x0020) or \
                (self.eeprom[0x09] == 0 and self.eeprom[0x0A] == 0):
            return 0
        else:
            return 1

    def read_eeprom_from_device(self):
        """
        Perform consecutive reads of the EEPROM to ensure a correct read, sets self.eeprom as a list of 16 bit
        integers read in big endian
        :returns: nothing
        :raises: ValueError - error during read from chip
        """
        first_read, status = self.device.i2c_read(0x2400, self.eeprom_size)
        if status != 0:
            raise ValueError("Error during initial read of eeprom")
        first_read = list(struct.unpack(">" + str(self.eeprom_size // 2) + "H", first_read))

        for m in range(2):
            consecutive_read, status = self.device.i2c_read(0x2400, self.eeprom_size)
            if status != 0:
                raise ValueError("Error during consecutive read of eeprom")
            consecutive_read = list(struct.unpack(">" + str(self.eeprom_size // 2) + "H", consecutive_read))

            for i in range(self.eeprom_size // 2):
                first_read[i] |= consecutive_read[i]
        self.eeprom = first_read
        self.get_bad_pixels()

    def get_parameter_code(self, param_id: ParameterCodesEEPROM, index=None):
        """
        Gets a named parameter from the eeprom. If it is an indexed parameter the index will be checked.
        :param ParameterCodesEEPROM param_id: the id of the parameter
        :param int index: only needed for indexed parameters
        :return: nothing
        :raises: ValueError - missing eeprom (not read from device), or the eeprom parameter is not defined
                 AttributeError - index not provided when needed
                 IndexError - index is out of predefined range for parameter
        """
        if self.eeprom is None:
            raise ValueError("EEPROM is not read from device")
        params = Mlx90641EEPROM.eeprom_map[param_id]
        if type(params) is int:
            return self.eeprom[params]
        elif type(params) is list:
            return self.get_bits(*params)
        elif type(params) is tuple:
            if index is None:
                raise AttributeError("For this parameter index can not be None")

            if params[0] <= index < params[1]:
                calculated_params = params[2](index)
                return self.get_bits(*calculated_params)
            else:
                raise IndexError("index: {} out of range ({}, {})".format(index, params[0], params[1]))
        else:
            ValueError("invalid eeprom parameter at {}".format(param_id))

    def get_bad_pixels(self):
        self.bad_pixels = []
        for pixel_i in range(192):
            if self.eeprom[0x0040 + pixel_i] == 0:
                if self.eeprom[0x0100 + pixel_i] == 0:
                    if self.eeprom[0x01C0 + pixel_i] == 0:
                        if self.eeprom[0x0680 + pixel_i] == 0:
                            self.bad_pixels.append(pixel_i)

        return self.bad_pixels


def main():
    # demo to test the file.. get a single frame from first comport found that has a EVB90640.
    # Should you want a more user friendly example, please have a look at Mlx9064x_dump_frame.py example.

    import time
    # work-around for having the parent directory added to the search path of packages.
    # to make `import mlx.pympt` to work, even EVB pip package itself is not installed!
    # note: when installed, it will take the installed version!
    # https://chrisyeh96.github.io/2017/08/08/definitive-guide-python-imports.html#case-2-syspath-could-change
    import os
    import sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

    if 10:
      from mlx.hw_usb_evb90640 import HwUsbEvb90640
      hw = HwUsbEvb90640()
    else:
      from mlx.hw_rpi_gpio_bitbang import HwRpiGpioBitBang
      hw = HwRpiGpioBitBang()

    # defaults
    frame_rate = 2.0
    max_frames = 10

    dev = Mlx9064x(hw, frame_rate=frame_rate)
    dev.init()

    frame_count = 0
    while frame_count < max_frames:
        frame = None
        try:
            frame = dev.read_frame()
        except Exception as e:
            dev.clear_error(frame_rate)
            print(e)
            pass

        if frame is not None:
            frame = dev.do_compensation(frame)
            print(",".join(map("{:6.2f}".format, frame)))
            frame_count += 1
        time.sleep(0.1)


if __name__ == '__main__':
    main()
