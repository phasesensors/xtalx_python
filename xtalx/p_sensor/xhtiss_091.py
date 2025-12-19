# Copyright (c) 2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import btype

from .xti import Measurement
from .cal_page import CalPage


class FrequencyResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_hz    = btype.float64_t()
    temperature_hz = btype.float64_t()
    _EXPECTED_SIZE = 17


class ConversionResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_psi   = btype.float64_t()
    temperature_c  = btype.float64_t()
    _EXPECTED_SIZE = 17


class FixedResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_psi   = btype.int32_t()
    temperature_c  = btype.int32_t()
    _EXPECTED_SIZE = 9


class FullResponse(btype.Struct, endian='<'):
    age_ms         = btype.uint8_t()
    pressure_psi   = btype.float64_t()
    temperature_c  = btype.float64_t()
    pressure_hz    = btype.float64_t()
    temperature_hz = btype.float64_t()
    _EXPECTED_SIZE = 33


class Comms:
    '''
    Communication protocol for sensor firmware 0.9.1 or below.
    '''
    def __init__(self, xhtiss):
        self.xhtiss = xhtiss

    def _set_nvstore(self, addr, data):
        assert len(data) == 4
        cmd = bytes([0x1E, 0x00, (addr & 0xFF), ((addr >> 8) & 0xFF),
                     data[0], data[1], data[2], data[3]])
        self.xhtiss.bus.transact(cmd)

    def _get_nvstore(self, addr, size):
        cmd = bytes([0x2A, 0x00, (addr & 0xFF), ((addr >> 8) & 0xFF), 0x00])
        cmd += bytes(size)
        data = self.xhtiss.bus.transact(cmd)
        return data[5:]

    def _read_ids(self):
        cmd = bytes([0x2A, 0x00, 0x01, 0xCA, 0x00]) + bytes(24)
        data = self.xhtiss.bus.transact(cmd)
        serial_number = data[5:].decode().strip('\x00')

        cmd = bytes([0x2A, 0x00, 0x02, 0xCA, 0x00]) + bytes(10)
        data = self.xhtiss.bus.transact(cmd)
        fw_version_str = data[5:].decode().strip('\x00')
        parts          = fw_version_str.split('.')
        fw_version = ((int(parts[0]) << 8) |
                      (int(parts[1]) << 4) |
                      (int(parts[2]) << 0))

        cmd = bytes([0x2A, 0x00, 0x03, 0xCA, 0x00]) + bytes(48)
        data = self.xhtiss.bus.transact(cmd)
        git_sha1 = data[5:].decode().strip('\x00')

        return serial_number, fw_version_str, fw_version, git_sha1

    def exec_cmd(self, cmd, rsp_len):
        cmd_bytes = bytes([cmd, 0x00]) + b'\x00'*rsp_len
        return self.xhtiss.bus.transact(cmd_bytes)[2:]

    def get_flash_params(self):
        data = self.xhtiss.bus.transact(b'\x2A\x00\xEF\xC0\x00\x00\x00\x00\x00')
        t_c = data[5]
        p_c = data[6]
        sample_ms = (data[8] << 8) | (data[7] << 0)
        return t_c, p_c, sample_ms

    def set_flash_params(self, t_c, p_c, sample_ms):
        cmd = bytes([0x1E, 0x00, 0xEF, 0xC0, t_c, p_c,
                     sample_ms & 0xFF, (sample_ms >> 8) & 0xFF])
        self.xhtiss.bus.transact(cmd)

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for the single calibration page stored in
        flash.
        '''
        data = b''
        for i in range(CalPage.get_short_size() // 4):
            address = 0x2000 + i*4
            rsp = self.xhtiss.bus.transact(bytes([
                0x2A, 0x00, (address >> 0) & 0xFF, (address >> 8) & 0xFF,
                0x00, 0x00, 0x00, 0x00, 0x00]))
            data += rsp[5:]
        pad = b'\xff' * (CalPage._EXPECTED_SIZE - len(data))
        return (data + pad,)

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
        return m
