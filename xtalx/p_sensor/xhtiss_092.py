# Copyright (c) 2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time
from enum import IntEnum

import btype

from .xti import Measurement
from .cal_page import CalPage
from .exception import XtalXException


CRC_0x9B_LUT = [
    0x00, 0x9B, 0xAD, 0x36, 0xC1, 0x5A, 0x6C, 0xF7,
    0x19, 0x82, 0xB4, 0x2F, 0xD8, 0x43, 0x75, 0xEE,
    0x32, 0xA9, 0x9F, 0x04, 0xF3, 0x68, 0x5E, 0xC5,
    0x2B, 0xB0, 0x86, 0x1D, 0xEA, 0x71, 0x47, 0xDC,
    0x64, 0xFF, 0xC9, 0x52, 0xA5, 0x3E, 0x08, 0x93,
    0x7D, 0xE6, 0xD0, 0x4B, 0xBC, 0x27, 0x11, 0x8A,
    0x56, 0xCD, 0xFB, 0x60, 0x97, 0x0C, 0x3A, 0xA1,
    0x4F, 0xD4, 0xE2, 0x79, 0x8E, 0x15, 0x23, 0xB8,
    0xC8, 0x53, 0x65, 0xFE, 0x09, 0x92, 0xA4, 0x3F,
    0xD1, 0x4A, 0x7C, 0xE7, 0x10, 0x8B, 0xBD, 0x26,
    0xFA, 0x61, 0x57, 0xCC, 0x3B, 0xA0, 0x96, 0x0D,
    0xE3, 0x78, 0x4E, 0xD5, 0x22, 0xB9, 0x8F, 0x14,
    0xAC, 0x37, 0x01, 0x9A, 0x6D, 0xF6, 0xC0, 0x5B,
    0xB5, 0x2E, 0x18, 0x83, 0x74, 0xEF, 0xD9, 0x42,
    0x9E, 0x05, 0x33, 0xA8, 0x5F, 0xC4, 0xF2, 0x69,
    0x87, 0x1C, 0x2A, 0xB1, 0x46, 0xDD, 0xEB, 0x70,
    0x0B, 0x90, 0xA6, 0x3D, 0xCA, 0x51, 0x67, 0xFC,
    0x12, 0x89, 0xBF, 0x24, 0xD3, 0x48, 0x7E, 0xE5,
    0x39, 0xA2, 0x94, 0x0F, 0xF8, 0x63, 0x55, 0xCE,
    0x20, 0xBB, 0x8D, 0x16, 0xE1, 0x7A, 0x4C, 0xD7,
    0x6F, 0xF4, 0xC2, 0x59, 0xAE, 0x35, 0x03, 0x98,
    0x76, 0xED, 0xDB, 0x40, 0xB7, 0x2C, 0x1A, 0x81,
    0x5D, 0xC6, 0xF0, 0x6B, 0x9C, 0x07, 0x31, 0xAA,
    0x44, 0xDF, 0xE9, 0x72, 0x85, 0x1E, 0x28, 0xB3,
    0xC3, 0x58, 0x6E, 0xF5, 0x02, 0x99, 0xAF, 0x34,
    0xDA, 0x41, 0x77, 0xEC, 0x1B, 0x80, 0xB6, 0x2D,
    0xF1, 0x6A, 0x5C, 0xC7, 0x30, 0xAB, 0x9D, 0x06,
    0xE8, 0x73, 0x45, 0xDE, 0x29, 0xB2, 0x84, 0x1F,
    0xA7, 0x3C, 0x0A, 0x91, 0x66, 0xFD, 0xCB, 0x50,
    0xBE, 0x25, 0x13, 0x88, 0x7F, 0xE4, 0xD2, 0x49,
    0x95, 0x0E, 0x38, 0xA3, 0x54, 0xCF, 0xF9, 0x62,
    0x8C, 0x17, 0x21, 0xBA, 0x4D, 0xD6, 0xE0, 0x7B,
]


def crc8(data, csum=0xFF):
    for v in data:
        csum = CRC_0x9B_LUT[(csum ^ v) & 0xFF]
    return csum


class SPIErrorCode(IntEnum):
    OK          = 0
    BAD_LENGTH  = 1
    BAD_CSUM    = 2


class FrequencyResponse091(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_hz    = btype.float64_t()
    temperature_hz = btype.float64_t()
    _EXPECTED_SIZE = 17


class ConversionResponse091(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_psi   = btype.float64_t()
    temperature_c  = btype.float64_t()
    _EXPECTED_SIZE = 17


class FixedResponse091(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_psi   = btype.int32_t()
    temperature_c  = btype.int32_t()
    _EXPECTED_SIZE = 9


class FullResponse091(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_psi   = btype.float64_t()
    temperature_c  = btype.float64_t()
    pressure_hz    = btype.float64_t()
    temperature_hz = btype.float64_t()
    _EXPECTED_SIZE = 33


class FrequencyResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    status         = btype.uint8_t()
    pressure_hz    = btype.float64_t()
    temperature_hz = btype.float64_t()
    _EXPECTED_SIZE = 18


class ConversionResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    status         = btype.uint8_t()
    pressure_psi   = btype.float64_t()
    temperature_c  = btype.float64_t()
    _EXPECTED_SIZE = 18


class FixedResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    status         = btype.uint8_t()
    pressure_psi   = btype.int32_t()
    temperature_c  = btype.int32_t()
    _EXPECTED_SIZE = 10


class FullResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    status         = btype.uint8_t()
    pressure_psi   = btype.float64_t()
    temperature_c  = btype.float64_t()
    pressure_hz    = btype.float64_t()
    temperature_hz = btype.float64_t()
    _EXPECTED_SIZE = 34


class ProtocolError(XtalXException):
    '''
    The protocol itself had garbage data.
    '''


class OpcodeMismatchError(XtalXException):
    '''
    The snesor thought it received a different command than the one we sent.
    '''


class RXChecksumError(XtalXException):
    '''
    We received a bad checksum in the response from the sensor.
    '''


class PrevCommandChecksumError(XtalXException):
    '''
    The previous command failed due to the sensor receiving a command with a
    bad checksum.
    '''


class PrevCommandUnrecognizedError(XtalXException):
    '''
    The previous command failed with a status code that we don't recognize.
    '''
    def __init__(self, err_code):
        super().__init__()
        self.err_code = err_code


class XHTISS_092:
    '''
    This is a driver for the high-temperature XHTISS sensor with an SPI
    interface, running firmware 0.9.2 or higher.
    '''
    def __init__(self, bus):
        self.bus          = bus
        self._halt_yield  = True
        self.last_time_ns = 0

        # print('a')
        self._synchronize()

        # print('b')
        self.nop(corrupt_csum=1)
        assert self._read_err() == SPIErrorCode.BAD_CSUM

        # print('c')
        try:
            self.nop()
        except PrevCommandChecksumError:
            pass
        except Exception as exc:
            raise Exception('Should have had a checksum error!') from exc
        assert self._read_err() == SPIErrorCode.OK

        # print('d')
        (self.serial_num,
         self.fw_version_str,
         self.fw_version,
         self.git_sha1) = self._read_ids()
        if self.fw_version < 0x092:
            raise Exception('Unsupported firmware version %s.'
                            % self.fw_version_str)

        self.cal_page = self.read_valid_calibration_page()

        self.poll_interval_sec = 0

        # print('e')
        self.report_id = None
        self.poly_psi  = None
        self.poly_temp = None
        if self.cal_page is not None:
            self.report_id = self.cal_page.get_report_id()
            self.poly_psi, self.poly_temp = self.cal_page.get_polynomials()

    def __str__(self):
        return 'XHTISS(%s)' % self.serial_num

    def _csum_transact(self, cmd, corrupt_csum=0):
        tx_csum = crc8(cmd) + corrupt_csum
        tx_cmd  = cmd + bytes([tx_csum])
        # print('TX: [%u] %s' % (len(tx_cmd), tx_cmd.hex()))
        data    = self.bus.transact(tx_cmd)
        # print('RX: [%u] %s %s' % (len(data), data.hex(), data))
        if data[0] != 0xAA:
            raise ProtocolError()
        if data[2] != cmd[0]:
            raise OpcodeMismatchError()

        rsp      = data[:-1]
        exp_csum = crc8(rsp)
        if exp_csum != data[-1]:
            # print('Expected CSUM: 0x%02X' % exp_csum)
            # print('Received CSUM: 0x%02X' % data[-1])
            raise RXChecksumError()

        return rsp[3:]

    def _read_err(self):
        tx_cmd = b'\x00\x00'
        # print('ETX: %s' % tx_cmd.hex())
        data = self.bus.transact(tx_cmd)
        # print('ERX: %s' % data.hex())
        if data[0] != 0xAA:
            raise ProtocolError()
        return data[1]

    def _synchronize(self):
        tx_cmd = b'\x34\x00\x00'
        tx_cmd = tx_cmd + bytes([crc8(tx_cmd)])
        while True:
            # print('STX: %s' % tx_cmd.hex())
            rsp = self.bus.transact(tx_cmd)
            # print('SRX: %s' % rsp.hex())
            if rsp[0] != 0xAA:
                continue
            if rsp[1] == 0xBB:
                raise Exception('Unsupported firmware version.')
            if rsp[1] != 0x00:
                continue
            if rsp[2] != 0x34:
                continue
            if rsp[3] != crc8(rsp[0:3]):
                continue

            # cmd = b'\x00\x00'
            # print('STX: %s' % cmd.hex())
            rsp = self.bus.transact(b'\x00\x00')
            # print('SRX: %s' % rsp.hex())
            if rsp[0] != 0xAA:
                continue
            if rsp[1] != 0x00:
                continue

            return 0

    def _read_ids(self):
        cmd = bytes([0x2A, 0x00, 0x00, 0x01, 0xCA, 0x00]) + bytes(24)
        data = self._csum_transact(cmd)
        if data[3] == 0xFF or data[3] == 0x00:
            raise Exception('Invalid ID response from sensor, may not be '
                            'connected or powered.')
        serial_number = data[3:].decode().strip('\x00')

        cmd = bytes([0x2A, 0x00, 0x00, 0x02, 0xCA, 0x00]) + bytes(10)
        data = self._csum_transact(cmd)
        fw_version_str = data[3:].decode().strip('\x00')
        parts          = fw_version_str.split('.')
        fw_version = ((int(parts[0]) << 8) |
                      (int(parts[1]) << 4) |
                      (int(parts[2]) << 0))

        cmd = bytes([0x2A, 0x00, 0x00, 0x03, 0xCA, 0x00]) + bytes(48)
        data = self._csum_transact(cmd)
        git_sha1 = data[3:].decode().strip('\x00')

        return serial_number, fw_version_str, fw_version, git_sha1

    def exec_cmd(self, cmd, rsp_len):
        cmd_bytes = bytes([cmd, 0x00, 0x00]) + b'\x00'*rsp_len
        return self._csum_transact(cmd_bytes)

    def nop(self, corrupt_csum=0):
        self._csum_transact(b'\x34\x00\x00', corrupt_csum=corrupt_csum)

    def get_flash_params(self):
        data = self._csum_transact(b'\x2A\x00\x00\xEF\xC0\x00\x00\x00\x00\x00')
        t_c = data[3]
        p_c = data[4]
        sample_ms = (data[8] << 8) | (data[7] << 0)
        return t_c, p_c, sample_ms

    def set_flash_params(self, t_c, p_c, sample_ms):
        cmd = bytes([0x1E, 0x00, 0x00, 0xEF, 0xC0, t_c, p_c,
                     sample_ms & 0xFF, (sample_ms >> 8) & 0xFF])
        self._csum_transact(cmd)

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for the single calibration page stored in
        flash.
        '''
        data = b''
        for i in range(CalPage.get_short_size() // 4):
            address = 0x2000 + i*4
            rsp = self._csum_transact(bytes([
                0x2A, 0x00, 0x00, (address >> 0) & 0xFF, (address >> 8) & 0xFF,
                0x00, 0x00, 0x00, 0x00, 0x00]))
            data += rsp[3:]
        pad = b'\xff' * (CalPage._EXPECTED_SIZE - len(data))
        return (data + pad,)

    def read_calibration_pages(self):
        '''
        Returns a CalPage struct for the single calibration page in sensor
        flash, even if the page is missing or corrupt.
        '''
        (cp_data,) = self.read_calibration_pages_raw()
        cp = CalPage.unpack(cp_data)
        return (cp,)

    def read_valid_calibration_page(self):
        '''
        Returns CalPage struct from the sensor flash.  Returns None if the
        calibration is not present or corrupted.
        '''
        (cp,) = self.read_calibration_pages()
        return cp if cp.is_valid() else None

    def read_frequencies(self):
        data = self.exec_cmd(0x19, FrequencyResponse._STRUCT.size)
        return FrequencyResponse.unpack(data)

    def read_conversion(self):
        data = self.exec_cmd(0x07, ConversionResponse._STRUCT.size)
        return ConversionResponse.unpack(data)

    def read_fixed(self):
        data = self.exec_cmd(0x33, FixedResponse._STRUCT.size)
        return FixedResponse.unpack(data)

    def read_full(self):
        data = self.exec_cmd(0x2D, FullResponse._STRUCT.size)
        # print(data)
        # print(len(data))
        return FullResponse.unpack(data)

    def read_measurement(self):
        rsp = self.read_full()

        m = Measurement(self, None, rsp.pressure_psi, rsp.temperature_c,
                        rsp.pressure_hz, rsp.temperature_hz, None, None, None,
                        None, None, None, None, None, None)
        m._age_ms = rsp.age_ms
        return m

    def yield_measurements(self, poll_interval_sec=None, **_kwargs):
        if poll_interval_sec is None:
            poll_interval_sec = self.poll_interval_sec

        self._halt_yield = False
        while not self._halt_yield:
            m = self.read_measurement()
            if m._age_ms > 25:
                continue

            yield m
            time.sleep(poll_interval_sec)

    def halt_yield(self):
        self._halt_yield = True

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
