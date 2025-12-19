# Copyright (c) 2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
from enum import IntEnum

import btype

from .xti import Measurement
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
    def __init__(self, tx_cmd, data, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tx_cmd = tx_cmd
        self.data = data

    def __str__(self):
        return 'ProtocolError(tx_cmd="%s", data="%s")' % (
                self.tx_cmd.hex(), self.data.hex())


class OpcodeMismatchError(XtalXException):
    '''
    The snesor thought it received a different command than the one we sent.
    '''
    def __init__(self, tx_cmd, data, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tx_cmd = tx_cmd
        self.data = data

    def __str__(self):
        return 'OpcodeMismatchError(tx_cmd="%s", data="%s")' % (
                self.tx_cmd.hex(), self.data.hex())


class RXChecksumError(XtalXException):
    '''
    We received a bad checksum in the response from the sensor.
    '''
    def __init__(self, tx_cmd, data, exp_csum, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tx_cmd = tx_cmd
        self.data = data
        self.exp_csum = exp_csum

    def __str__(self):
        return 'RXChecksumError(tx_cmd="%s", data="%s")' % (
                self.tx_cmd.hex(), self.data.hex())


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


class Comms:
    '''
    Communication protocol for sensor firmare 0.9.2 or later.
    '''
    def __init__(self, xhtiss):
        self.xhtiss = xhtiss

        self._synchronize()

        self.nop(corrupt_csum=1)
        assert self._read_err() == SPIErrorCode.BAD_CSUM

        try:
            self.nop()
        except PrevCommandChecksumError:
            pass
        except Exception as exc:
            raise Exception('Should have had a checksum error!') from exc
        assert self._read_err() == SPIErrorCode.OK

    def _csum_transact(self, cmd, corrupt_csum=0):
        tx_csum = crc8(cmd) + corrupt_csum
        tx_cmd  = cmd + bytes([tx_csum])
        data    = self.xhtiss.bus.transact(tx_cmd)
        if data[0] != 0xAA:
            raise ProtocolError(tx_cmd, data)
        if data[2] != cmd[0]:
            raise OpcodeMismatchError(tx_cmd, data)

        rsp      = data[:-1]
        exp_csum = crc8(rsp)
        if exp_csum != data[-1]:
            raise RXChecksumError(tx_cmd, data, exp_csum)

        return rsp[3:]

    def _read_err(self):
        tx_cmd = b'\x00\x00'
        data = self.xhtiss.bus.transact(tx_cmd)
        if data[0] != 0xAA:
            raise ProtocolError(tx_cmd, data)
        return data[1]

    def _synchronize(self):
        tx_cmd = b'\x34\x00\x00'
        tx_cmd = tx_cmd + bytes([crc8(tx_cmd)])
        while True:
            rsp = self.xhtiss.bus.transact(tx_cmd)
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

            rsp = self.xhtiss.bus.transact(b'\x00\x00')
            if rsp[0] != 0xAA:
                continue
            if rsp[1] != 0x00:
                continue

            return 0

    def _set_nvstore(self, addr, data):
        assert len(data) == 4
        cmd = bytes([0x1E, 0x00, 0x00, (addr & 0xFF), ((addr >> 8) & 0xFF),
                     data[0], data[1], data[2], data[3]])
        self._csum_transact(cmd)

    def _get_nvstore(self, addr, size):
        cmd = bytes([0x2A, 0x00, 0x00, (addr & 0xFF), ((addr >> 8) & 0xFF),
                     0x00])
        cmd += bytes(size)
        data = self._csum_transact(cmd)
        return data[3:]

    def exec_cmd(self, cmd, rsp_len):
        cmd_bytes = bytes([cmd, 0x00, 0x00]) + b'\x00'*rsp_len
        return self._csum_transact(cmd_bytes)

    def nop(self, corrupt_csum=0):
        self._csum_transact(b'\x34\x00\x00', corrupt_csum=corrupt_csum)

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
        return FullResponse.unpack(data)

    def read_measurement(self):
        rsp = self.read_full()

        m = Measurement(self.xhtiss, None, rsp.pressure_psi, rsp.temperature_c,
                        rsp.pressure_hz, rsp.temperature_hz, None, None, None,
                        None, None, None, None, None, None)
        m._age_ms = rsp.age_ms
        m._status = rsp.status
        return m
