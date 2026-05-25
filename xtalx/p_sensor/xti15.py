# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
from enum import IntEnum

import btype

import xtalx.usbcmd


CMD_EP       = 0x01
RSP_EP       = 0x81
TELEMETRY_EP = 0x82


class Opcode(IntEnum):
    SET_SAMPLE_INTERVAL_MS  = 0x100
    SET_IIR_ALPHA_SHIFT     = 0x101
    GET_T_POLY_PARAMS       = 0x102
    SET_T_POLY_PARAMS       = 0x103
    GET_P_POLY_PARAMS       = 0x104
    SET_P_POLY_PARAMS       = 0x105

    ERASE_FLASH_PARAMS      = 0x2FD
    ERASE_RAM_PARAMS        = 0x2FE
    SAVE_PARAMS             = 0x2FF


class TelemetryPacket(btype.Struct, endian='<'):
    seq_num         = btype.uint32_t()
    flags           = btype.uint32_t()
    temp_hz         = btype.float64_t()
    temp_c          = btype.float64_t()
    pressure_hz     = btype.float64_t()
    pressure_psi    = btype.float64_t()
    _EXPECTED_SIZE  = 40


class Measurement:
    '''
    Object encapsulating the results of an XTI sensor measurement.  The
    following fields are defined:

        sensor - Reference to the XTI that generated the Measurement.
        temp_freq - Measured temperature crystal frequency.
        temp_c - Temperature measured in degrees Celsius.
        pressure_freq - Measured pressure crystal frequency.
        pressure_psi - Temperature-compensated pressure measured in PSI.
        flags - A set of validity and error flags.

    If the sensor is uncalibrated then temp_c and pressure_psi will be None.
    '''
    def __init__(self, sensor, seq_num, flags, temp_freq, temp_c,
                 pressure_freq, pressure_psi, time_ns=None):
        self.sensor             = sensor
        self.seq_num            = seq_num
        self.flags              = flags
        self.temp_freq          = temp_freq
        self.temp_c             = temp_c
        self.pressure_freq      = pressure_freq
        self.pressure_psi       = pressure_psi
        self.time_ns            = time_ns or sensor.time_ns_increasing()

    @staticmethod
    def _from_packet(sensor, packet):
        tp = TelemetryPacket.unpack(packet)
        return Measurement(sensor, tp.seq_num, tp.flags, tp.temp_hz, tp.temp_c,
                           tp.pressure_hz, tp.pressure_psi)

    def tostring(self, verbose=False):
        s = '%s: ' % self.sensor
        if verbose:
            s += ('F 0x%04X tf %s t %s pf %s p %s' %
                  (self.flags, self.temp_freq, self.temp_c, self.pressure_freq,
                   self.pressure_psi))
        else:
            s += '%s PSI, %s C' % (self.pressure_psi, self.temp_c)

        return s

    def to_stsdb_point(self, time_ns=None):
        time_ns = time_ns or self.sensor.time_ns_increasing()
        p = {
            'time_ns'                   : time_ns,
            'pressure_psi'              : self.pressure_psi,
            'temp_c'                    : self.temp_c,
            'pressure_freq_hz'          : self.pressure_freq,
            'temp_freq_hz'              : self.temp_freq,
            'lores_pressure_psi'        : None,
            'lores_temp_c'              : None,
            'lores_pressure_freq_hz'    : None,
            'lores_temp_freq_hz'        : None,
        }
        return p


class XTI15(xtalx.usbcmd.Device):
    def __init__(self, usb_dev):
        super().__init__(usb_dev, CMD_EP, RSP_EP, 256, 256, git_sha1_index=6,
                         default_configuration=0x68)

    def __str__(self):
        return 'XTI(%s)' % self.serial_num

    def read_measurement(self, timeout=2000):
        '''
        Synchronously read a single measurement from the sensor, blocking if no
        measurement is currently available.
        '''
        p = self.usb_dev.read(TELEMETRY_EP, 64, timeout=timeout)
        return Measurement._from_packet(self, p)
