# Copyright (c) 2026 by Phase Advanced Sensor Systems Corp.
import time
import struct
import math
from enum import IntEnum

import btype

import xtalx.usbcmd
from xtalx.tools.math import PolynomialFit1D


CMD_EP       = 0x01
RSP_EP       = 0x81
TELEMETRY_EP = 0x83


class Status(IntEnum):
    FLASH_ERASE_FAILED      = 100
    FLASH_WRITE_FAILED      = 101


class Opcode(IntEnum):
    SET_SAMPLE_INTERVAL_MS  = 0x100
    SET_IIR_ALPHA_SHIFT     = 0x101
    GET_T_POLY_PARAMS       = 0x102
    SET_T_POLY_PARAMS       = 0x103

    ERASE_FLASH_PARAMS      = 0x2FD
    ERASE_RAM_PARAMS        = 0x2FE
    SAVE_PARAMS             = 0x2FF


class TelemetryPacket(btype.Struct):
    seq_num        = btype.uint32_t()
    flags          = btype.uint32_t()
    temp_hz        = btype.float64_t()
    temp_c         = btype.float64_t()
    _EXPECTED_SIZE = 24


class Measurement:
    '''
    Object encapsulating the results of an XTP sensor measurement.  The
    following fields are defined:

        sensor - Reference to the XTP that generated the Measurement.
        temp_freq - Measured temperature crystal frequency.
        temp_c - Temperature measured in degrees Celsius.
        flags - A set of validity and error flags.

    If the sensor is uncalibrated then temp_c will be None.
    '''
    def __init__(self, sensor, seq_num, flags, temp_freq, temp_c,
                 time_ns=None):
        self.sensor    = sensor
        self.seq_num   = seq_num
        self.flags     = flags
        self.temp_freq = temp_freq
        self.temp_c    = temp_c
        self.time_ns   = time_ns or sensor.time_ns_increasing()

    @staticmethod
    def _from_packet(sensor, packet):
        tp = TelemetryPacket.unpack(packet)
        return Measurement(sensor, tp.seq_num, tp.flags, tp.temp_hz, tp.temp_c)

    def tostring(self, verbose=False):
        s = '%s: ' % self.sensor
        if verbose:
            s = ('F 0x%04X tf %s t %s' %
                 (self.flags, self.temp_freq, self.temp_c))
        else:
            s = '%s C' % self.temp_c

        return s

    def to_stsdb_point(self, time_ns=None):
        return {
            'time_ns'      : self.time_ns if time_ns is None else time_ns,
            'temp_freq_hz' : self.temp_freq,
            'temp_c'       : self.temp_c,
        }


class XTP(xtalx.usbcmd.Device):
    def __init__(self, usb_dev):
        super().__init__(usb_dev, CMD_EP, RSP_EP, 256, 256, git_sha1_index=6,
                         default_configuration=0x64)

    def __str__(self):
        return 'XTP(%s)' % self.serial_num

    def log(self, tag, s, timestamp=None):
        timestamp = timestamp or time.time_ns()
        print('[%u - %s] %s: %s' % (timestamp, self.serial_num, tag, s))

    def set_sample_interval_ms(self, period_ms):
        '''
        Sets the sample rate for the sensor telemetry endpoint.  This updates
        the RAM copy of the parameter and causes the sensor to start reporting
        at the new rate.  Changing the sample rate does not affect the sensor
        resolution or accuracy.  The default value is 100 ms.
        '''
        self._exec_command(Opcode.SET_SAMPLE_INTERVAL_MS, [period_ms])

    def set_iir_alpha_shift(self, alpha_shift):
        '''
        Sets the IIR filter alpha shift (2**-alpha_shift) value.  This updates
        the RAM copy of the parameter and causes the sensor to start reporting
        values using the new filter coefficient.  A larger filter coefficient
        helps reduce sensor noise, but can slow down the response time to a
        fast temperature change.  The default value is 13.
        '''
        self._exec_command(Opcode.SET_IIR_ALPHA_SHIFT, [alpha_shift])

    def get_t_poly(self):
        '''
        Returns the temperature polynomial stored in flash, if the sensor has
        been calibrated.  Returns None otherwise.
        '''
        _, data = self._exec_command(Opcode.GET_T_POLY_PARAMS)
        if not data:
            return None

        one, order, k1, k2 = struct.unpack_from('<IIdd', data, offset=0)
        assert one == 1

        offset = 24
        coefs = []
        for _ in range(order + 1):
            coefs.append(struct.unpack_from('<d', data, offset=offset)[0])
            offset += 8

        return PolynomialFit1D.from_k1_k2_coefs(k1, k2, coefs)

    def set_t_poly(self, t_poly):
        '''
        Updates the temperature polynomial stored in flash with the new value.
        Unlike the RAM parameters, this is updated in flash immediately and the
        sensor will start emitting measurements using the new polynomial on the
        next telemetry transmit.

        The t_poly argument can be specified as None to delete any existing
        polynomial from flash.
        '''
        if t_poly is None:
            data = b''
            param = 0x01CDEDBD
        else:
            data = t_poly.to_poly_class_double_efficient(5).pack()
            param = 0xBA1ECABB
        self._exec_command(Opcode.SET_T_POLY_PARAMS,
                           [param, math.ceil(time.time())],
                           data=data)

    def erase_flash_params(self):
        '''
        Erases the flash copy of the sensor parameters, resetting it to default
        values.  The RAM copy is unaffected and continues to be used by the
        sensor until the sensor is rebooted at which point the default flash
        values will be used.
        '''
        self._exec_command(Opcode.ERASE_FLASH_PARAMS, [0xC0CAC01A])

    def erase_ram_params(self):
        '''
        Erases the RAM copy of the sensor parameters, resetting it to default
        values.  The sensor starts to report measurements according to the
        default values.  The flash copy is unaffected, so the sensor will switch
        back to using the previously-saved values upon reboot.
        '''
        self._exec_command(Opcode.ERASE_RAM_PARAMS, [0xC1E0D0D0])

    def save_params(self):
        '''
        Copies the sensor parameters from RAM into flash, preserving them so
        that they will be automatically used when the sensor boots up in the
        future.
        '''
        self._exec_command(Opcode.SAVE_PARAMS, [0xCDEDBDBE])

    def read_measurement(self, timeout=2000):
        '''
        Synchronously read a single measurement from the sensor, blocking if no
        measurement is currently available.
        '''
        p = self.usb_dev.read(TELEMETRY_EP, 64, timeout=timeout)
        return Measurement._from_packet(self, p)
