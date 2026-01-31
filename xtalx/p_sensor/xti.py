# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import threading
import random
import errno
import struct
import time
from enum import IntEnum

import usb
import usb.util

import btype

from .exception import XtalXException
from .cal_page import CalPage


FC_FLAGS_VALID              = (1 << 15)
FC_FLAG_NO_TEMP_PRESSURE    = (1 << 4)
FC_FLAG_PRESSURE_FAILED     = (1 << 3)
FC_FLAG_TEMP_FAILED         = (1 << 2)
FC_FLAG_PRESSURE_UPDATE     = (1 << 1)
FC_FLAG_TEMP_UPDATE         = (1 << 0)

FC_FLAG_114_NO_PRESSURE_PSI = (1 << 4)
FC_FLAG_114_HAVE_PID_INFO   = (1 << 5)
FC_FLAG_114_NO_TEMP_C       = (1 << 6)


class Status(IntEnum):
    OK              = 0
    BAD_LENGTH      = 1
    BAD_OPCODE      = 2
    NOT_FOUND       = 3

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            s = '%s' % Status(rsp.status)
        except ValueError:
            s = '%u' % rsp.status
        return s


class CommandException(Exception):
    def __init__(self, rsp, rx_data):
        super().__init__(
            'Command exception: %s (%s)' % (Status.rsp_to_status_str(rsp), rsp))
        self.rsp = rsp
        self.rx_data = rx_data


class Opcode(IntEnum):
    # System commands.
    GET_INFO        = 0x0001
    GET_PID_INFO    = 0x0002
    SET_PID_INFO    = 0x0003
    DISABLE_PID     = 0x0004
    ENABLE_PID      = 0x0005
    SET_DAC_VALUE   = 0x0006

    # Flash commands.
    FLASH_READ_CAL  = 0xBA61

    # Illegal opcode.
    BAD_OPCODE      = 0xCCCC


class CommandHeader(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    flags           = btype.uint32_t()
    _EXPECTED_SIZE  = 8


class SetPIDInfoPayload(btype.Struct):
    Kp              = btype.float64_t()
    Ki              = btype.float64_t()
    Kd              = btype.float64_t()
    setpoint_c      = btype.float64_t()
    _EXPECTED_SIZE  = 32


class SetDACValuePayload(btype.Struct):
    dac_value       = btype.float64_t()
    _EXPECTED_SIZE  = 8


class Response(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    status          = btype.uint32_t()
    params          = btype.Array(btype.uint32_t(), 12)
    _EXPECTED_SIZE  = 56


class GetInfoResponse(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    status          = btype.uint32_t()
    flags           = btype.uint32_t()
    f_hs_mhz        = btype.uint32_t()
    hclk            = btype.float64_t()
    _EXPECTED_SIZE  = 24


class GetPIDInfoResponse(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    status          = btype.uint32_t()
    Kp              = btype.float64_t()
    Ki              = btype.float64_t()
    Kd              = btype.float64_t()
    setpoint_c      = btype.float64_t()
    dac_t_val       = btype.float64_t()
    _EXPECTED_SIZE  = 48


class FrequencyPacket24(btype.Struct):
    '''
    Firmware revisions 1.0.6 and earlier return a 24-byte packet if the sensor
    doesn't have enough data to perform a temperature-compensated pressure
    measurement yet or if the sensor doesn't have a calibration applied in
    flash.
    '''
    ref_freq            = btype.uint32_t()
    pressure_edges      = btype.uint32_t()
    pressure_ref_clocks = btype.uint32_t()
    temp_edges          = btype.uint32_t()
    temp_ref_clocks     = btype.uint32_t()
    flags               = btype.uint16_t()
    seq_num             = btype.uint8_t()
    rsrv                = btype.uint8_t()
    _EXPECTED_SIZE      = 24


class FrequencyPacket40(btype.Struct):
    '''
    Firmware revisions 1.0.6 and earlier return a 40-byte packet if the sensor
    has enough data to perform a temperature-compensated pressure measurement.
    '''
    ref_freq            = btype.uint32_t()
    pressure_edges      = btype.uint32_t()
    pressure_ref_clocks = btype.uint32_t()
    temp_edges          = btype.uint32_t()
    temp_ref_clocks     = btype.uint32_t()
    flags               = btype.uint16_t()
    seq_num             = btype.uint8_t()
    rsrv                = btype.uint8_t()
    pressure_psi        = btype.float64_t()
    temp_c              = btype.float64_t()
    _EXPECTED_SIZE      = 40


class FrequencyPacket56(btype.Struct):
    '''
    Firmware revisions 1.0.7 and higher always return a 56-byte packet that
    contains flags indicating the validity of things like the temperature-
    compensated pressure measurement.  These firmware versions also return the
    MCU temperature as a control.
    '''
    ref_freq            = btype.uint32_t()
    pressure_edges      = btype.uint32_t()
    pressure_ref_clocks = btype.uint32_t()
    temp_edges          = btype.uint32_t()
    temp_ref_clocks     = btype.uint32_t()
    flags               = btype.uint16_t()
    seq_num             = btype.uint8_t()
    rsrv                = btype.uint8_t()
    pressure_psi        = btype.float64_t()
    temp_c              = btype.float64_t()
    mcu_temp_c          = btype.float64_t()
    rsrv2               = btype.Array(btype.uint8_t(), 8)
    _EXPECTED_SIZE      = 56


class FrequencyPacket60_110(btype.Struct):
    '''
    Firmware revision 1.1.0 redoes the entire packet contents, while
    maintaining backwards-compatibility for the essential fields.
    '''
    cP                     = btype.float32_t()
    cI                     = btype.float32_t()
    cD                     = btype.float32_t()
    dac_2p20               = btype.uint32_t()
    pressure_hz_1e4        = btype.uint32_t()
    flags                  = btype.uint16_t()
    seq_num                = btype.uint8_t()
    rsrv1                  = btype.uint8_t()
    pressure_psi           = btype.float64_t()
    temp_c                 = btype.float64_t()
    temp_hz_1e4            = btype.uint32_t()
    lores_pressure_hz_1e4  = btype.uint32_t()
    lores_temp_hz_1e4      = btype.uint32_t()
    lores_pressure_psi     = btype.float32_t()
    lores_temp_c           = btype.float32_t()
    _EXPECTED_SIZE         = 60


class FrequencyPacket60_114(btype.Struct):
    '''
    Firmware revision 1.1.4 removes lores fields and keeps only the IIR field
    output now.  It also removes many flags and introduces mode fields for both
    the P and T crystals.  Finally, frequencies are stored using double-
    precision values since we have extra space from removing lores support.
    The pressure_psi and temp_c fields are in the same place.
    '''
    cP                     = btype.float32_t()
    cI                     = btype.float32_t()
    cD                     = btype.float32_t()
    dac_2p20               = btype.uint32_t()
    rsrv0                  = btype.uint32_t()
    flags                  = btype.uint16_t()
    seq_num                = btype.uint8_t()
    pt_mode                = btype.uint8_t()
    pressure_psi           = btype.float64_t()
    temp_c                 = btype.float64_t()
    temp_hz                = btype.float64_t()
    pressure_hz            = btype.float64_t()
    rsrv1                  = btype.uint32_t()
    _EXPECTED_SIZE         = 60


class TIMMode(IntEnum):
    INITIALIZING    = 0
    OVERFLOW        = 1
    UNDERFLOW       = 2
    RUNNING         = 3


class Measurement:
    '''
    Object encapsulating the results of an XTI sensor measurement.  The
    following fields are defined:

        sensor - Reference to the XTI that generated the Measurement.
        ref_freq - Frequency of the sensor's reference crystal.
        pressure_edges - Number of pressure crystal ticks used to generate the
            Measurement.
        pressure_ref_clocks - Number of reference clock ticks that elapsed
            while counting pressure_edges pressure crystal ticks.
        pressure_freq - Measured pressure crystal frequency.
        temp_edges - Number of temperature crystal ticks used to generate the
            Measurement.
        temp_ref_clocks - Number of temperature crystal ticks that elapsed
            while counting temp_edges temperature crystal ticks.
        temp_freq - Measured temperature crystal frequency.
        mcu_temp_c - Microcontroller's internal junction temperature.
        pressure_psi - Temperature-compensated pressure measured in PSI.
        temp_c - Temperature measured in degrees Celsius.
        flags - A set of validity and error flags.

    If the sensor is uncalibrated or has not sampled enough data to generate
    a temperature-compensated pressure measurement then some or all of
    temp_freq, pressure_freq, pressure_psi and temp_c may be None.

    The flags field is a bitmask which may include any of the following bits;
    it may be None if the firmware version predates the introduction of status
    flags:

        FC_FLAGS_VALID - The flags field contains valid information (always set
            or flags will be None).
        FC_FLAG_NO_TEMP_PRESSURE - Will be set if pressure_psi and temp_c could
            not be generated; the sensor may be uncalibrated or may not have
            generated both temperature and pressure crystal readings yet.
        FC_FLAG_PRESSURE_FAILED - Will be set if 0.5 seconds elapse without a
            pressure crystal measurement completing; this indicates that a
            sensor failure has caused the pressure crystal to stop ticking.
        FC_FLAG_TEMP_FAILED - Will be set if 0.5 seconds elapse without a
            temperature crystal measurement completing; this indicates that a
            sensor failure has caused the temperature crystal to stop ticking.
        FC_FLAG_PRESSURE_UPDATE - Indicates that the current Measurement
            incorporates a new reading from the pressure crystal; it may still
            be incorporating the previous reading from the temperature crystal.
        FC_FLAG_TEMP_UPDATE - Indicates that the current Measurement
            incorporates a new reading from the temperature crystal; it may
            still be incorporating the previous reading from the pressure
            crystal.

    Note that since the temperature and pressure crystals tick asynchronously
    with respect to one another, a measurement on one crystal is likely to
    complete while a measurement on the other crystal is still pending and so
    typically only one of FC_FLAG_PRESSURE_UPDATE or FC_FLAG_TEMP_UPDATE will
    be set.
    '''
    def __init__(self, sensor, mcu_temp_c, pressure_psi, temp_c, pressure_freq,
                 temp_freq, lores_pressure_psi, lores_temp_c,
                 lores_pressure_freq, lores_temp_freq, cP, cI, cD, dac, flags,
                 seq_num=None):
        self.sensor              = sensor
        self.mcu_temp_c          = mcu_temp_c
        self.pressure_psi        = pressure_psi
        self.temp_c              = temp_c
        self.pressure_freq       = pressure_freq
        self.temp_freq           = temp_freq
        self.lores_pressure_psi  = lores_pressure_psi
        self.lores_temp_c        = lores_temp_c
        self.lores_pressure_freq = lores_pressure_freq
        self.lores_temp_freq     = lores_temp_freq
        self.cP                  = cP
        self.cI                  = cI
        self.cD                  = cD
        self.dac                 = dac
        self.flags               = flags
        self.seq_num             = seq_num

    @staticmethod
    def _from_packet(sensor, packet):
        mt, p, t, Fp, Ft = None, None, None, None, None
        lp, lt, Flp, Flt = None, None, None, None
        cP, cI, cD, dac  = None, None, None, None
        seq_num          = None

        if sensor.usb_dev.bcdDevice < 0x0107:
            if len(packet) == 24:
                fp = FrequencyPacket24.unpack(packet)
            else:
                fp = FrequencyPacket40.unpack(packet)
                p  = fp.pressure_psi
                t  = fp.temp_c
                Fp = fp.ref_freq*fp.pressure_edges/fp.pressure_ref_clocks
                Ft = fp.ref_freq*fp.temp_edges/fp.temp_ref_clocks
        elif sensor.usb_dev.bcdDevice < 0x0110:
            fp = FrequencyPacket56.unpack(packet)
            mt = fp.mcu_temp_c
            assert fp.flags and (fp.flags & FC_FLAGS_VALID)
            if not (fp.flags & FC_FLAG_PRESSURE_FAILED):
                Fp = fp.ref_freq*fp.pressure_edges/fp.pressure_ref_clocks
            if not (fp.flags & FC_FLAG_TEMP_FAILED):
                Ft = fp.ref_freq*fp.temp_edges/fp.temp_ref_clocks
            if (fp.flags & FC_FLAG_NO_TEMP_PRESSURE) == 0:
                p = fp.pressure_psi
                t = fp.temp_c
        elif sensor.usb_dev.bcdDevice < 0x0114:
            fp = FrequencyPacket60_110.unpack(packet)
            assert fp.flags and (fp.flags & FC_FLAGS_VALID)
            if not (fp.flags & FC_FLAG_PRESSURE_FAILED):
                Fp  = fp.pressure_hz_1e4 / 1e4
                Flp = fp.lores_pressure_hz_1e4 / 1e4
            if not (fp.flags & FC_FLAG_TEMP_FAILED):
                Ft  = fp.temp_hz_1e4 / 1e4
                Flt = fp.lores_temp_hz_1e4 / 1e4
            if (fp.flags & FC_FLAG_NO_TEMP_PRESSURE) == 0:
                p   = fp.pressure_psi
                t   = fp.temp_c
                lp  = fp.lores_pressure_psi
                lt  = fp.lores_temp_c
                if sensor.is_pid_supported():
                    cP  = fp.cP
                    cI  = fp.cI
                    cD  = fp.cD
                    dac = fp.dac_2p20 / 2**20
        else:
            fp = FrequencyPacket60_114.unpack(packet)
            assert fp.flags and (fp.flags & FC_FLAGS_VALID)
            p_mode = ((fp.pt_mode >> 2) & 0x3)
            t_mode = ((fp.pt_mode >> 0) & 0x3)
            if p_mode == TIMMode.RUNNING:
                Fp = fp.pressure_hz
            if t_mode == TIMMode.RUNNING:
                Ft = fp.temp_hz
            if (fp.flags & FC_FLAG_114_NO_PRESSURE_PSI) == 0:
                p = fp.pressure_psi
            if (fp.flags & FC_FLAG_114_NO_TEMP_C) == 0:
                t = fp.temp_c
            if (fp.flags & FC_FLAG_114_HAVE_PID_INFO):
                cP  = fp.cP
                cI  = fp.cI
                cD  = fp.cD
                dac = fp.dac_2p20 / 2**20
            seq_num = fp.seq_num

        flags = fp.flags if fp.flags & FC_FLAGS_VALID else None

        return Measurement(sensor, mt, p, t, Fp, Ft, lp, lt, Flp, Flt, cP, cI,
                           cD, dac, flags, seq_num)

    def tostring(self, verbose=False):
        s = '%s: ' % self.sensor
        if verbose:
            if self.flags is not None:
                s += 'F 0x%04X ' % self.flags
            if hasattr(self, '_current_amps'):
                s += 'mA %.2f ' % (self._current_amps * 1000)
            age_ms = getattr(self, '_age_ms', None)
            if age_ms is not None:
                s += 'ms %u ' % age_ms
            status = getattr(self, '_status', None)
            if status is not None:
                s += 'status 0x%02X ' % status
            s += ('pf %s tf %s p %s t %s lpf %s ltf %s lp %s lt %s mt %s' %
                  (self.pressure_freq, self.temp_freq,
                   self.pressure_psi, self.temp_c, self.lores_pressure_freq,
                   self.lores_temp_freq, self.lores_pressure_psi,
                   self.lores_temp_c, self.mcu_temp_c))
        else:
            if self.pressure_psi is None:
                p = 'n/a'
            else:
                p = '%f' % self.pressure_psi

            if self.temp_c is None:
                t = 'n/a'
            else:
                t = '%f' % self.temp_c
            s += '%s PSI, %s C' % (p, t)

        return s

    def to_influx_point(self, time_ns=None, measurement='xtalx_data',
                        fields=None):
        time_ns = time_ns or time.time_ns()
        fields  = fields or {}
        p = {
            'measurement' : measurement,
            'time'        : time_ns,
            'tags'        : {'sensor' : self.sensor.serial_num},
            'fields'      : fields,
        }
        if self.mcu_temp_c is not None:
            fields['mcu_temp_c'] = float(self.mcu_temp_c)
        if self.pressure_psi is not None:
            fields['pressure_psi'] = float(self.pressure_psi)
        if self.temp_c is not None:
            fields['temp_c'] = float(self.temp_c)
        if self.pressure_freq is not None:
            fields['pressure_freq_hz'] = float(self.pressure_freq)
        if self.temp_freq is not None:
            fields['temp_freq_hz'] = float(self.temp_freq)
        if self.lores_pressure_psi is not None:
            fields['lores_pressure_psi'] = float(self.lores_pressure_psi)
        if self.lores_temp_c is not None:
            fields['lores_temp_c'] = float(self.lores_temp_c)
        if self.lores_pressure_freq is not None:
            fields['lores_pressure_freq_hz'] = float(self.lores_pressure_freq)
        if self.lores_temp_freq is not None:
            fields['lores_temp_freq_hz'] = float(self.lores_temp_freq)
        if self.cP is not None:
            fields['cP']  = float(self.cP)
            fields['cI']  = float(self.cI)
            fields['cD']  = float(self.cD)
            fields['dac'] = float(self.dac)
        return p

    def to_stsdb_points(self, time_ns=None):
        time_ns = time_ns or self.sensor.time_ns_increasing()
        p = {
            'time_ns'          : time_ns,
            'pressure_psi'     : self.pressure_psi,
            'temp_c'           : self.temp_c,
            'pressure_freq_hz' : self.pressure_freq,
            'temp_freq_hz'     : self.temp_freq,
        }
        lp = {
            'time_ns'          : time_ns,
            'pressure_psi'     : self.lores_pressure_psi,
            'temp_c'           : self.lores_temp_c,
            'pressure_freq_hz' : self.lores_pressure_freq,
            'temp_freq_hz'     : self.lores_temp_freq,
        }
        return p, lp

    def to_combined_stsdb_point(self, time_ns=None):
        time_ns = time_ns or self.sensor.time_ns_increasing()
        return {
            'time_ns'                : time_ns,
            'pressure_psi'           : self.pressure_psi,
            'temp_c'                 : self.temp_c,
            'pressure_freq_hz'       : self.pressure_freq,
            'temp_freq_hz'           : self.temp_freq,
            'lores_pressure_psi'     : self.lores_pressure_psi,
            'lores_temp_c'           : self.lores_temp_c,
            'lores_pressure_freq_hz' : self.lores_pressure_freq,
            'lores_temp_freq_hz'     : self.lores_temp_freq,
        }

    def to_lhp_stsdb_point(self, time_ns=None):
        time_ns = time_ns or self.sensor.time_ns_increasing()
        return {
            'time_ns'   : time_ns,
            'cP'        : self.cP,
            'cI'        : self.cI,
            'cD'        : self.cD,
            'dac'       : self.dac,
        }


class XTI:
    TELEMETRY_EP    = 0x81
    CMD_EP          = 0x02
    RSP_EP          = 0x83

    '''
    Given a USB device handle acquired via find() or find_one(), creates an
    XTI object that can be used to communicate with a sensor.
    '''
    def __init__(self, usb_dev):
        self.usb_dev      = usb_dev
        self.lock         = threading.RLock()
        self._halt_yield  = True
        self.thread       = None
        self.last_time_ns = 0

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 6)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise XtalXException(
                    'Device has no langid, ensure running as root!') from e
        self.fw_version_str = (str((self.fw_version >> 8) & 0xF) + '.' +
                               str((self.fw_version >> 4) & 0xF) + '.' +
                               str((self.fw_version >> 0) & 0xF))

        if self.usb_dev.bcdDevice >= 0x0110:
            self._synchronize()
            self.tag = random.randint(1, 0xFFFF)
            self.xinfo = self._get_info()
            if self.xinfo.flags & (1 << 0):
                self.pinfo = self._get_pid_info()
            else:
                self.pinfo = None

        self.report_id = None
        self.poly_psi  = None
        self.poly_temp = None
        self.cal_page  = None
        if self.usb_dev.bcdDevice >= 0x0114:
            cp0_data, cp1_data = self.read_calibration_pages_raw()
            self._cal_pages = [CalPage.unpack(cp0_data),
                               CalPage.unpack(cp1_data)]
            if self._cal_pages[0].is_valid():
                self.cal_page = self._cal_pages[0]
            elif self._cal_pages[1].is_valid():
                self.cal_page = self._cal_pages[1]

        if self.cal_page is not None:
            self.report_id = self.cal_page.get_report_id()
            self.poly_psi, self.poly_temp = self.cal_page.get_polynomials()

        if self.report_id is None:
            try:
                self.report_id = int(usb.util.get_string(usb_dev, 15))
            except ValueError:
                pass

        self.usb_path = '%s:%s' % (
            usb_dev.bus, '.'.join('%u' % n for n in usb_dev.port_numbers))

    def __str__(self):
        return 'XTI(%s)' % self.serial_num

    def _set_configuration(self, bConfigurationValue):
        with self.lock:
            cfg = None
            try:
                cfg = self.usb_dev.get_active_configuration()
            except usb.core.USBError as e:
                if e.strerror != 'Configuration not set':
                    raise

            if cfg is None or cfg.bConfigurationValue != bConfigurationValue:
                usb.util.dispose_resources(self.usb_dev)
                self.usb_dev.set_configuration(bConfigurationValue)

    def _set_measurement_config(self):
        self._set_configuration(2)

    def _synchronize(self):
        self._set_measurement_config()
        while True:
            try:
                self.usb_dev.read(self.RSP_EP, 16384, timeout=100)
            except usb.core.USBTimeoutError:
                break

    def _alloc_tag(self):
        tag      = self.tag
        self.tag = 1 if self.tag == 0xFFFF else self.tag + 1
        return tag

    def _exec_command(self, opcode, flags=0, params=b'', bulk_data=b'',
                      timeout=1000, rx_len=0):
        params = params + bytes(48 - len(params))

        tag  = self._alloc_tag()
        hdr  = CommandHeader(opcode=opcode, tag=tag, flags=flags)
        data = hdr.pack() + params + bulk_data
        l    = self.usb_dev.write(self.CMD_EP, data, timeout=timeout)
        assert l == len(data)

        data = self.usb_dev.read(self.RSP_EP, Response._STRUCT.size + rx_len,
                                 timeout=timeout)
        assert len(data) >= Response._STRUCT.size
        rsp_bytes = data[:Response._STRUCT.size]
        rsp = Response.unpack(rsp_bytes)
        assert rsp.opcode == opcode
        assert rsp.tag    == tag

        rx_data = bytes(data[Response._STRUCT.size:])
        if rsp.status != Status.OK:
            raise CommandException(rsp, rx_data)

        assert len(rx_data) == rx_len
        return rsp, rsp_bytes, rx_data

    def _get_info(self):
        _, rsp_bytes, _ = self._exec_command(Opcode.GET_INFO)
        return GetInfoResponse.unpack_from(rsp_bytes)

    def _get_pid_info(self):
        _, rsp_bytes, _ = self._exec_command(Opcode.GET_PID_INFO)
        return GetPIDInfoResponse.unpack_from(rsp_bytes)

    def _set_pid_info(self, Kp=None, Ki=None, Kd=None, setpoint_c=None):
        flags = 0
        if Kp is not None:
            flags |= (1 << 0)
        if Ki is not None:
            flags |= (1 << 1)
        if Kd is not None:
            flags |= (1 << 2)
        if setpoint_c is not None:
            flags |= (1 << 3)
        payload = SetPIDInfoPayload(Kp=(Kp or 0), Ki=(Ki or 0), Kd=(Kd or 0),
                                    setpoint_c=(setpoint_c or 0))
        return self._exec_command(Opcode.SET_PID_INFO, flags, payload.pack())[0]

    def _disable_pid(self):
        return self._exec_command(Opcode.DISABLE_PID)[0]

    def _enable_pid(self):
        return self._exec_command(Opcode.ENABLE_PID)[0]

    def _set_dac_value(self, dac_value):
        return self._exec_command(
                Opcode.SET_DAC_VALUE,
                params=SetDACValuePayload(dac_value=dac_value).pack())[0]

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for each of the two calibration pages stored
        in flash.  Returns a tuple:

            (cal_data_0, cal_data_1)
        '''
        if self.usb_dev.bcdDevice < 0x0114:
            return None, None

        _, _, cal_data_0 = self._exec_command(Opcode.FLASH_READ_CAL,
                                              params=struct.pack('<I', 0),
                                              rx_len=2048)
        _, _, cal_data_1 = self._exec_command(Opcode.FLASH_READ_CAL,
                                              params=struct.pack('<I', 1),
                                              rx_len=2048)
        return cal_data_0, cal_data_1

    def read_calibration_pages(self):
        '''
        Returns CalPage structs for each calibration page in the sensor flash,
        even if the page(s) are missing or corrupt.
        '''
        if self.usb_dev.bcdDevice < 0x0114:
            return None, None

        cp0_data, cp1_data = self.read_calibration_pages_raw()
        cp0 = CalPage.unpack(cp0_data)
        cp1 = CalPage.unpack(cp1_data)
        return cp0, cp1

    def read_valid_calibration_page(self):
        '''
        Returns CalPage struct from the sensor flash.  Returns None if the
        calibration is not present or both pages are corrupted.
        '''
        if self.usb_dev.bcdDevice < 0x0114:
            return None

        cp0, cp1 = self.read_calibration_pages()
        return cp0 if cp0.is_valid() else cp1 if cp1.is_valid() else None

    def is_pid_supported(self):
        return self.pinfo is not None

    def read_measurement(self, timeout=2000):
        '''
        Synchronously read a single measurement from the sensor, blocking if no
        measurement is currently available.
        '''
        with self.lock:
            p = self.usb_dev.read(self.TELEMETRY_EP, 64, timeout=timeout)
        return Measurement._from_packet(self, p)

    def _yield_measurements(self, do_reset, timeout):
        with self.lock:
            if do_reset:
                self.usb_dev.reset()
            self._set_measurement_config()

            while not self._halt_yield:
                try:
                    yield self.read_measurement(timeout=timeout)
                except usb.core.USBError as e:
                    if e.errno != errno.ETIMEDOUT:
                        raise
                    continue

    def yield_measurements(self, do_reset=True, timeout=2000):
        '''
        Yields Measurement objects synchronously in the current thread,
        blocking while waiting for new measurements to be acquired.
        '''
        with self.lock:
            self._halt_yield = False
            yield from self._yield_measurements(do_reset, timeout=timeout)

    def halt_yield(self):
        '''
        Halts an ongoing yield_measurements() call, causing it to eventually
        terminate the generator loop.
        '''
        self._halt_yield = True

    def _read_measurements_async(self, handler, do_reset, timeout):
        with self.lock:
            for m in self._yield_measurements(do_reset, timeout=timeout):
                handler(m)

    def read_measurements(self, handler, do_reset=True, timeout=2000):
        '''
        Reads measurements asynchronously in a separate thread, calling the
        handler as measurements become available.  The handler should take a
        single Measurement object as an argument.
        '''
        with self.lock:
            assert self.thread is None
            self._halt_yield = False
            self.thread = threading.Thread(target=self._read_measurements_async,
                                           args=(handler, do_reset, timeout),
                                           daemon=False)
            self.thread.start()

    def join_read(self):
        '''
        Blocks the current thread until the asynchronous read thread completes.
        Typically this blocks indefinitely until some error occurs, however the
        read thread will also exit if someone sets the _halt_yield field to
        True (see XTI.halt_read()).
        '''
        self.thread.join()

    def halt_read(self):
        '''
        Halts any asynchronous measurement thread and waits for it to finish
        cleanly.
        '''
        self._halt_yield = True
        self.join_read()

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
