# Copyright (c) 2022 Phase Advanced Sensor Systems, Inc.
from enum import IntEnum
import random
import time
import math
import struct

import usb
import usb.util
import btype

from xtalx.tools.iter import prange
from .cal_page import CalPage


class Status(IntEnum):
    OK              = 0
    BAD_LENGTH      = 1
    BAD_OPCODE      = 2
    QSPI_BUSY       = 3

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            s = '%s' % Status(rsp.status)
        except ValueError:
            s = '%u' % rsp.status
        return s


class Opcode(IntEnum):
    # System commands.
    GET_BOOT_STATUS          = 0x0001
    TIMESTAMP                = 0x0002
    GET_VOLTAGES             = 0x0003
    SET_AUTONOMOUS_CONFIG    = 0x0004
    START_CONTINUOUS_MODE    = 0x0005
    START_AUTONOMOUS_MODE    = 0x0006
    CLEAR_ERASE_JOURNAL      = 0x0007
    CLEAR_AUTO_JOURNAL       = 0x0008
    CLEAR_FLASH_LOG          = 0x0009
    GET_AUTONOMOUS_CONFIG    = 0x0010
    READ_FLASH_LOG           = 0x0011
    STOP_CONTINUOUS_MODE     = 0x0012

    # QSPI flash commands.
    GET_QSPI_FLASH_INFO      = 0x0100
    READ_QSPI_FLASH          = 0x0101
    START_ERASE_QSPI_FLASH   = 0x0102
    GET_QSPI_ERASE_STATUS    = 0x0103

    # Flash commands.
    FLASH_READ_CAL           = 0xBA61


class Command(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    params          = btype.Array(btype.uint32_t(), 7)
    _EXPECTED_SIZE  = 32


class Response(btype.Struct):
    opcode         = btype.uint16_t()
    tag            = btype.uint16_t()
    status         = btype.uint32_t()
    params         = btype.Array(btype.uint32_t(), 6)
    _EXPECTED_SIZE = 32


class CalPoint(btype.Struct):
    seq             = btype.uint32_t()
    vdd_mv          = btype.uint16_t()
    vbus_mv         = btype.uint16_t()
    ft              = btype.float64_t()
    fp              = btype.float64_t()
    lft             = btype.float64_t()
    lfp             = btype.float64_t()
    _EXPECTED_SIZE  = 40


class Measurement:
    def __init__(self, sensor, t_ns, seq, vdd_mv, vbus_mv, ft, fp, lft, lfp):
        self.sensor = sensor
        self.seq    = seq
        self.t_ns   = t_ns
        self.vdd    = vdd_mv / 1000
        self.vbus   = vbus_mv / 1000
        self.ft     = ft if not math.isnan(ft) else 0
        self.fp     = fp if not math.isnan(fp) else 0
        self.lft    = lft if not math.isnan(lft) else 0
        self.lfp    = lfp if not math.isnan(lfp) else 0

        self.pressure_psi = None
        self.lores_pressure_psi = None
        if sensor.poly_psi is not None:
            if self.ft and self.fp:
                self.pressure_psi = sensor.poly_psi(self.fp, self.ft)
            if self.lft and self.lfp:
                self.lores_pressure_psi = sensor.poly_psi(self.lfp, self.lft)

        self.temp_c = None
        self.lores_temp_c = None
        if sensor.poly_temp is not None:
            if self.ft:
                self.temp_c = sensor.poly_temp(self.ft)
            if self.lft:
                self.lores_temp_c = sensor.poly_temp(self.lft)

    @staticmethod
    def from_cal_point(sensor, t_ns, cp):
        return Measurement(sensor, t_ns, cp.seq, cp.vdd_mv, cp.vbus_mv, cp.ft,
                           cp.fp, cp.lft, cp.lfp)

    def to_influx_point(self, time_ns=None, measurement='xtalx_data',
                        fields=None):
        fields = fields or {}
        p = {
            'measurement' : measurement,
            'time'        : time_ns or self.t_ns,
            'tags'        : {'sensor' : self.sensor.serial_num},
            'fields'      :
            {
                'temp_freq_hz'           : float(self.ft),
                'pressure_freq_hz'       : float(self.fp),
                'lores_temp_freq_hz'     : float(self.lft),
                'lores_pressure_freq_hz' : float(self.lfp),
                **fields,
            },
        }
        if self.pressure_psi is not None:
            fields['pressure_psi'] = float(self.pressure_psi)
        if self.temp_c is not None:
            fields['temp_c'] = self.temp_c
        if self.lores_pressure_psi is not None:
            fields['lores_pressure_psi'] = float(self.lores_pressure_psi)
        if self.lores_temp_c is not None:
            fields['lores_temp_c'] = self.lores_temp_c
        return p

    def to_combined_stsdb_point(self, time_ns=None):
        return {
            'time_ns'                : time_ns or self.t_ns,
            'pressure_psi'           : self.pressure_psi,
            'temp_c'                 : self.temp_c,
            'pressure_freq_hz'       : self.fp,
            'temp_freq_hz'           : self.ft,
            'lores_pressure_psi'     : self.lores_pressure_psi,
            'lores_temp_c'           : self.lores_temp_c,
            'lores_pressure_freq_hz' : self.lfp,
            'lores_temp_freq_hz'     : self.lft,
        }


class CommandException(Exception):
    def __init__(self, rsp, rx_data):
        super().__init__(
            'Command exception: %s (%s)' % (Status.rsp_to_status_str(rsp), rsp))
        self.rsp = rsp
        self.rx_data = rx_data


class BootStatus:
    def __init__(self, post_result, pll_freq, hse_freq, ls_ticks, page_size):
        self.post_result    = post_result
        self.pll_freq       = pll_freq
        self.hse_freq       = hse_freq
        self.ls_ticks       = ls_ticks
        self.ls_freq        = pll_freq * 1024 / ls_ticks if ls_ticks else 0
        self.page_size      = page_size

    def __str__(self):
        return ('BootStatus(post_result=0x%08X, pll_freq=%u, hse_freq=%u, '
                'ls_ticks=%u)' %
                (self.post_result, self.pll_freq, self.hse_freq, self.ls_ticks))


class Voltages:
    def __init__(self, vdd_mv, vbus_mv):
        self.vdd_mv  = vdd_mv
        self.vbus_mv = vbus_mv

    def __str__(self):
        return 'Voltages(vdd_mv=%u, vbus_mv=%u)' % (self.vdd_mv, self.vbus_mv)


class AutonomousConfig:
    def __init__(self, interval_secs, flags):
        self.interval_secs = interval_secs
        self.flags         = flags


class QSPIInfo:
    def __init__(self, dev_id, m_id, d_id, write_index, nslots):
        self.dev_id      = dev_id
        self.m_id        = m_id
        self.d_id        = d_id
        self.write_index = write_index
        self.nslots      = nslots
        self.capacity    = (1 << (d_id & 0xFF))


class QSPIEraseStatus:
    def __init__(self, erase_in_progress, extended_read_register):
        self.erase_in_progress      = erase_in_progress
        self.extended_read_register = extended_read_register


class XMHTI:
    CMD_EP = 0x01
    RSP_EP = 0x82
    CAL_EP = 0x83

    def __init__(self, usb_dev):
        self.usb_dev      = usb_dev
        self.tag          = random.randint(0, 65535)
        self.last_time_ns = 0

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 6)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise Exception(
                    'Device has no langid, ensure running as root!') from e

        self.fw_version_str = '%u.%u.%u' % (
                (self.fw_version >> 8) & 0xFF,
                (self.fw_version >> 4) & 0x0F,
                (self.fw_version >> 0) & 0x0F)
        assert self.fw_version >= 0x092

        self._synchronize()
        self._boot_status = self.get_boot_status()

        cp0_data, cp1_data = self.read_calibration_pages_raw()
        self._cal_pages = [CalPage.unpack(cp0_data), CalPage.unpack(cp1_data)]
        if self._cal_pages[0].is_valid():
            self.cal_page = self._cal_pages[0]
        elif self._cal_pages[1].is_valid():
            self.cal_page = self._cal_pages[1]
        else:
            self.cal_page = None

        self.report_id = None
        self.poly_psi  = None
        self.poly_temp = None
        if self.cal_page is not None:
            self.report_id = self.cal_page.get_report_id()
            self.poly_psi, self.poly_temp = self.cal_page.get_polynomials()

    def __str__(self):
        return 'XMHTI(%s)' % self.serial_num

    def _exec_command(self, opcode, params=None, bulk_data=b'', timeout=1000,
                      rx_len=0):
        if not params:
            params = [0, 0, 0, 0, 0, 0, 0]
        elif len(params) < 7:
            params = params + [0]*(7 - len(params))

        tag      = self.tag
        self.tag = (self.tag + 1) & 0xFFFF

        cmd  = Command(opcode=opcode, tag=tag, params=params)
        data = cmd.pack()
        l    = self.usb_dev.write(self.CMD_EP, data + bulk_data,
                                  timeout=timeout)
        assert l == 32 + len(bulk_data)

        data = self.usb_dev.read(self.RSP_EP, Response._STRUCT.size + rx_len,
                                 timeout=timeout)
        assert len(data) >= Response._STRUCT.size
        rsp = Response.unpack(data[:Response._STRUCT.size])
        assert rsp.opcode == opcode
        assert rsp.tag    == tag

        rx_data = bytes(data[Response._STRUCT.size:])
        if rsp.status != Status.OK:
            raise CommandException(rsp, rx_data)

        assert len(rx_data) == rx_len
        return rsp, rx_data

    def _exec_qspi_read_command(self, opcode, index, nslots, timeout=1000):
        tag      = self.tag
        self.tag = (self.tag + 1) & 0xFFFF
        cmd      = Command(opcode=opcode, tag=tag,
                           params=[index, nslots, 0, 0, 0, 0, 0])
        data     = cmd.pack()
        self.usb_dev.write(self.CMD_EP, data, timeout=timeout)

        # Read the data and check for errors.  TODO: We can't discriminate
        # between reading 4 slots and reading a 32-byte error response.
        data = self.usb_dev.read(self.RSP_EP, nslots * 8, timeout=timeout)
        if len(data) != nslots * 8:
            assert len(data) == Response._STRUCT.size
            rsp = Response.unpack(data)
            assert rsp.tag == tag
            raise CommandException(rsp, b'')

        return bytes(data)

    def _set_configuration(self, bConfigurationValue):
        cfg = None
        try:
            cfg = self.usb_dev.get_active_configuration()
        except usb.core.USBError as e:
            if e.strerror != 'Configuration not set':
                raise

        if cfg is None or cfg.bConfigurationValue != bConfigurationValue:
            usb.util.dispose_resources(self.usb_dev)
            self.usb_dev.set_configuration(bConfigurationValue)

    def _synchronize(self):
        self._set_configuration(0x51)
        while True:
            try:
                self.usb_dev.read(self.RSP_EP, 16384, timeout=100)
            except usb.core.USBTimeoutError:
                break
        self.stop_continuous_mode()

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for each of the two calibration pages stored
        in flash.  Returns a tuple:

            (cal_data_0, cal_data_1)
        '''
        _, cal_data_0 = self._exec_command(Opcode.FLASH_READ_CAL, [0],
                                           rx_len=2048)
        _, cal_data_1 = self._exec_command(Opcode.FLASH_READ_CAL, [1],
                                           rx_len=2048)
        return cal_data_0, cal_data_1

    def read_calibration_pages(self):
        '''
        Returns CalPage structs for each calibration page in the sensor flash,
        even if the page(s) are missing or corrupt.
        '''
        cp0_data, cp1_data = self.read_calibration_pages_raw()
        cp0 = CalPage.unpack(cp0_data)
        cp1 = CalPage.unpack(cp1_data)
        return cp0, cp1

    def read_valid_calibration_page(self):
        '''
        Returns CalPage struct from the sensor flash.  Returns None if the
        calibration is not present or both pages are corrupted.
        '''
        cp0, cp1 = self.read_calibration_pages()
        return cp0 if cp0.is_valid() else cp1 if cp1.is_valid() else None

    def get_boot_status(self):
        rsp, _ = self._exec_command(Opcode.GET_BOOT_STATUS)
        return BootStatus(rsp.params[0], rsp.params[1], rsp.params[2] * 1000000,
                          rsp.params[3], rsp.params[4])

    def record_timestamp(self, unix_time=None):
        if unix_time is None:
            unix_time = int(time.time())
        self._exec_command(Opcode.TIMESTAMP, [unix_time])
        return unix_time

    def get_voltages(self):
        rsp, _ = self._exec_command(Opcode.GET_VOLTAGES)
        return Voltages(rsp.params[0], rsp.params[1])

    def set_autonomous_config(self, sample_interval_secs, flags):
        self._exec_command(Opcode.SET_AUTONOMOUS_CONFIG,
                           [sample_interval_secs, flags])

    def get_autonomous_config(self):
        rsp, _ = self._exec_command(Opcode.GET_AUTONOMOUS_CONFIG)
        return AutonomousConfig(rsp.params[0], rsp.params[1])

    def start_continuous_mode(self):
        self._exec_command(Opcode.START_CONTINUOUS_MODE)

    def stop_continuous_mode(self):
        self._exec_command(Opcode.STOP_CONTINUOUS_MODE)
        while True:
            try:
                self.usb_dev.read(self.CAL_EP, 16384, timeout=100)
            except usb.core.USBTimeoutError:
                break

    def start_autonomous_mode(self):
        self._exec_command(Opcode.START_AUTONOMOUS_MODE)

    def clear_erase_journal(self):
        self._exec_command(Opcode.CLEAR_ERASE_JOURNAL)

    def clear_auto_journal(self):
        self._exec_command(Opcode.CLEAR_AUTO_JOURNAL)

    def read_flash_log(self):
        tag      = self.tag
        self.tag = (self.tag + 1) & 0xFFFF
        cmd      = Command(opcode=Opcode.READ_FLASH_LOG, tag=tag)
        data     = cmd.pack()
        self.usb_dev.write(self.CMD_EP, data, timeout=1000)

        # Read page-sized chunks until we get a zero-length one to terminate
        # the transfer.
        data = b''
        while True:
            chunk = self.usb_dev.read(self.RSP_EP, self._boot_status.page_size,
                                      timeout=1000)
            if len(chunk) == Response._STRUCT.size:
                rsp = Response.unpack(chunk)
                assert rsp.tag == tag
                raise CommandException(rsp, b'')
            if len(chunk) == 0:
                break

            assert len(chunk) == self._boot_status.page_size
            data += chunk

        return data

    def clear_flash_log(self):
        self._exec_command(Opcode.CLEAR_FLASH_LOG)

    def get_qspi_flash_info(self):
        rsp, _ = self._exec_command(Opcode.GET_QSPI_FLASH_INFO)
        return QSPIInfo(rsp.params[0], rsp.params[1], rsp.params[2],
                        rsp.params[3], rsp.params[4])

    def read_qspi_flash(self, index, nslots):
        data = b''
        for i in prange(index, index + nslots, 16384):
            n       = min(nslots, 16384)
            data   += self._exec_qspi_read_command(Opcode.READ_QSPI_FLASH, i, n,
                                                   timeout=1000)
            nslots -= n
        return data

    def start_erase_qspi_flash(self):
        self._exec_command(Opcode.START_ERASE_QSPI_FLASH)

    def get_qspi_erase_status(self):
        rsp, _ = self._exec_command(Opcode.GET_QSPI_ERASE_STATUS)
        return QSPIEraseStatus(rsp.params[0], rsp.params[1])

    def _read_measurement(self, timeout=10):
        '''
        Reads a single block of 100 measurements.
        '''
        data    = b''
        rem     = 4008
        while rem:
            try:
                new_data = self.usb_dev.read(self.CAL_EP, rem, timeout=timeout)
                rem     -= len(new_data)
                data    += new_data
            except usb.core.USBTimeoutError:
                time.sleep(0.02)

        return data

    def read_measurements(self):
        '''
        Reads blocks of 100 measurements.  This uses a very aggressive timeout
        which breaks the read up into multiple shorter operations and seems to
        generate less USB noise on the XMHTI5.
        '''
        data = self._read_measurement(timeout=1)
        t_ns = time.time_ns()
        if t_ns <= self.last_time_ns:
            t_ns = self.last_time_ns + 1
        n_queued = struct.unpack('<I', data[:4])[0]
        assert ((len(data) - 8) % CalPoint._STRUCT.size) == 0
        N = (len(data) - 8) // CalPoint._STRUCT.size
        assert N == 100
        ms = [Measurement.from_cal_point(
                    self,
                    t_ns + (i - 100 - n_queued) * 100000000,
                    CalPoint.unpack_from(data, 8 + i * CalPoint._STRUCT.size)
                ) for i in range(N)]

        self.last_time_ns = ms[-1].t_ns

        if n_queued >= 100:
            return ms + self.read_measurements()

        return ms
