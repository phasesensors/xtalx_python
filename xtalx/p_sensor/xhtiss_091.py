# Copyright (c) 2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import btype

from .xti import Measurement


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

    def exec_cmd(self, cmd, rsp_len):
        cmd_bytes = bytes([cmd, 0x00]) + b'\x00'*rsp_len
        return self.xhtiss.bus.transact(cmd_bytes)[2:]

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
