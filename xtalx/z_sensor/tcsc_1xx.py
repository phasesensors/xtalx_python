# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import random
import time
from enum import IntEnum

import usb.core
import btype

from .tcsc_types import (DriveType, ResetReason, Opcode, GetInfoResponse,
                         SweepFit)


class StartCalPayload(btype.Struct):
    sigout          = btype.uint32_t()
    bias            = btype.uint32_t()
    _EXPECTED_SIZE  = 8


class EvalFreqsPayload(btype.Struct):
    temp_hz         = btype.float64_t()
    center_hz       = btype.float64_t()
    width_hz        = btype.float64_t()
    _EXPECTED_SIZE  = 24


class EvalFreqsResponse(btype.Struct):
    opcode                  = btype.uint16_t()
    tag                     = btype.uint16_t()
    status                  = btype.uint32_t()
    flags                   = btype.uint32_t()
    rsrv                    = btype.uint32_t()
    temp_c                  = btype.float64_t()
    density_g_per_ml        = btype.float64_t()
    viscosity_cp            = btype.float64_t()
    _EXPECTED_SIZE          = 40


class Status(IntEnum):
    OK              = 0
    BAD_OPCODE      = 1
    BAD_CMD_LENGTH  = 2
    ABORTED         = 3
    BUSY            = 4
    FAILED          = 5

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            s = '%s' % Status(rsp.status)
        except ValueError:
            s = '%u' % rsp.status
        return s


class CommandException(Exception):
    def __init__(self, rsp):
        super().__init__(
            'Command exception: %s (%s)' % (Status.rsp_to_status_str(rsp), rsp))
        self. rsp = rsp


class CommandHeader(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    rsrv            = btype.uint32_t()
    _EXPECTED_SIZE  = 8


class StartScoperPayload(btype.Struct):
    dds_skip        = btype.uint32_t()
    amplitude       = btype.uint32_t()
    _EXPECTED_SIZE  = 8


class AutoChirpPayload(btype.Struct):
    skip0           = btype.uint32_t()
    skip1           = btype.uint32_t()
    amplitude       = btype.uint32_t()
    _EXPECTED_SIZE  = 12


class SetTEnablePayload(btype.Struct):
    enabled         = btype.uint32_t()
    _EXPECTED_SIZE  = 4


class FitCommandPayload(btype.Struct):
    flags           = btype.uint32_t()
    cordic_rot      = btype.uint32_t()
    rsrv            = btype.Array(btype.uint8_t(), 8)
    temp_hz         = btype.float64_t()
    _EXPECTED_SIZE  = 24


class GenHiresFreqsPayload(btype.Struct):
    f0              = btype.float64_t()
    width           = btype.float64_t()
    N               = btype.uint32_t()
    _EXPECTED_SIZE  = 20


class Response(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    status          = btype.uint32_t()
    params          = btype.Array(btype.uint32_t(), 12)
    _EXPECTED_SIZE  = 56


class GetEInfoResponse(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    status          = btype.uint32_t()
    r_source        = btype.float64_t()
    r_feedback      = btype.float64_t()
    dac_to_v_coefs  = btype.Array(btype.float64_t(), 2)
    adc_to_v_coefs  = btype.Array(btype.float64_t(), 2)
    _EXPECTED_SIZE  = 56


class ReadTempResponse(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    status          = btype.uint32_t()
    cpu_ticks       = btype.uint64_t()
    osc_ticks       = btype.uint32_t()
    _EXPECTED_SIZE  = 20


class FitPointsResponse(btype.Struct):
    opcode                  = btype.uint16_t()
    tag                     = btype.uint16_t()
    status                  = btype.uint32_t()
    fit_flags               = btype.uint16_t()
    fit_niter               = btype.uint8_t()
    fit_status              = btype.int8_t()
    temp_c                  = btype.float32_t()
    fit_RR                  = btype.float32_t()
    strength                = btype.float32_t()
    density_g_per_ml        = btype.float64_t()
    viscosity_cp            = btype.float64_t()
    peak_hz                 = btype.float64_t()
    peak_fwhm               = btype.float64_t()
    _EXPECTED_SIZE          = 56


class StartSweepPayload(btype.Struct):
    nfreqs              = btype.uint32_t()
    amplitude           = btype.uint32_t()
    double_precision    = btype.uint32_t()
    _EXPECTED_SIZE      = 12


class Comms:
    def __init__(self, usb_dev):
        self.usb_dev = usb_dev

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 4)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise Exception(
                    'Device has no langid, ensure running as root!') from e

        if self.fw_version < 0x101:
            raise Exception('Firmware version of 0x%X not supported.' %
                            self.fw_version)
        if self.fw_version < 0x111:
            self.CMD_EP   = 0x01
            self.RSP_EP   = 0x82
            self.SCOPE_EP = 0x83
        else:
            self.CMD_EP   = 0x01
            self.RSP_EP   = 0x81
            self.SCOPE_EP = 0x83

        self.tag = random.randint(1, 0xFFFF)

        self._set_configuration(usb_dev, 0x15)
        self._synchronize()

        self.cmd_buf_len = self._get_info().cmd_buf_len

    @staticmethod
    def _set_configuration(usb_dev, bConfigurationValue, force=False):
        cfg = None
        try:
            cfg = usb_dev.get_active_configuration()
        except usb.core.USBError as e:
            if e.strerror != 'Configuration not set':
                raise

        if (cfg is None or cfg.bConfigurationValue != bConfigurationValue or
                force):
            usb.util.dispose_resources(usb_dev)
            usb_dev.set_configuration(bConfigurationValue)

    def _write_cmd(self, data, **kwargs):
        return self.usb_dev.write(self.CMD_EP, data, **kwargs)

    def _read_rsp(self, size, **kwargs):
        return self.usb_dev.read(self.RSP_EP, size, **kwargs)

    def _read_scope(self, size, **kwargs):
        return self.usb_dev.read(self.SCOPE_EP, size, **kwargs)

    def _send_abort(self):
        self._write_cmd(b'')

    def _alloc_tag(self):
        tag      = self.tag
        self.tag = 1 if self.tag == 0xFFFF else self.tag + 1
        return tag

    def _synchronize(self):
        self._send_abort()
        tag  = self._alloc_tag()
        hdr  = CommandHeader(opcode=Opcode.BAD_OPCODE, tag=tag)
        data = hdr.pack() + bytes(48)
        junk = 0
        while True:
            self._write_cmd(data)
            rsp = self._read_rsp(64, timeout=100)
            if len(rsp) == 56:
                break

            junk += len(rsp)

        self._write_cmd(data)
        data = self._read_rsp(64)
        assert len(data) == 56

        rsp = Response.unpack(data)
        assert rsp.tag    == tag
        assert rsp.opcode == Opcode.BAD_OPCODE
        assert rsp.status == Status.BAD_OPCODE

        junk_len = 0
        try:
            while True:
                junk_len += len(self._read_scope(64, timeout=100))
        except usb.core.USBTimeoutError:
            pass

        return junk

    def _send_command(self, opcode, params, bulk_data, timeout):
        tag  = self._alloc_tag()
        hdr  = CommandHeader(opcode=opcode, tag=tag)
        data = hdr.pack() + params + bytes(48 - len(params)) + bulk_data
        size = self._write_cmd(data, timeout=timeout)
        assert size == len(data)
        if (len(data) % 64) == 0:
            if len(data) < self.cmd_buf_len:
                self._write_cmd(b'')
        return tag

    def _recv_response(self, tag, timeout, cls=Response):
        data = self._read_rsp(Response._STRUCT.size, timeout=timeout)
        assert len(data) == Response._STRUCT.size
        rsp = cls.unpack_from(data)
        assert rsp.tag == tag

        if rsp.status != Status.OK:
            rsp.opcode = Opcode(rsp.opcode)
            raise CommandException(rsp)

        return rsp

    def _exec_command(self, opcode, params=b'', bulk_data=b'', timeout=1000,
                      cls=Response):
        tag = self._send_command(opcode, params, bulk_data, timeout)
        return self._recv_response(tag, timeout, cls=cls)

    def _get_info(self):
        rsp = self._exec_command(Opcode.GET_INFO)

        hclk                 = rsp.params[0]
        dclk_divisor         = rsp.params[1] >> 16
        aclk_divisor         = rsp.params[1] & 0xFFFF
        cmd_buf_len          = rsp.params[2] >> 10
        f_hs_mhz             = rsp.params[2] & 0x03FF
        drive_type           = DriveType(rsp.params[4] & 0xFF)
        reset_reason         = ResetReason((rsp.params[4] >> 8) & 0xFF)
        cal_params           = rsp.params[4] >> 16
        reset_csr            = rsp.params[5]
        reset_sr1            = rsp.params[6]
        max_sweep_entries    = rsp.params[7] & 0xFFFF
        cal_dac_amplitude    = rsp.params[7] >> 16
        electronics_cal_date = rsp.params[8]
        crystal_cal_date     = rsp.params[9]
        air_f0               = rsp.params[10] / 1000
        air_fwhm             = rsp.params[11] / 1000

        if self.fw_version < 0x106:
            nresets       = rsp.params[3]
            dv_nominal_hz = 32768
        else:
            nresets       = (rsp.params[3] >> 24) & 0xFF
            dv_nominal_hz = (rsp.params[3] & 0xFFFFFF) or 32768

        return GetInfoResponse(
                hclk, dclk_divisor, aclk_divisor, cmd_buf_len, f_hs_mhz,
                drive_type, reset_reason, cal_params, reset_csr,
                reset_sr1, max_sweep_entries, cal_dac_amplitude,
                electronics_cal_date, crystal_cal_date, air_f0, air_fwhm,
                nresets, dv_nominal_hz)

    def _get_einfo(self):
        return self._exec_command(Opcode.GET_EINFO, cls=GetEInfoResponse)

    def _send_scope_cmd(self, dds_skip, amplitude):
        self._exec_command(
            Opcode.START_SCOPER,
            StartScoperPayload(dds_skip=dds_skip, amplitude=amplitude).pack())

    def _send_auto_chirp_cmd(self, skip0, skip1, amplitude):
        self._exec_command(
            Opcode.AUTO_CHIRP,
            AutoChirpPayload(skip0=skip0, skip1=skip1,
                             amplitude=amplitude).pack())

    def _sweep_async(self, amplitude, freq_tuples, bulk_data):
        params = StartSweepPayload(nfreqs=len(freq_tuples),
                                   amplitude=amplitude,
                                   double_precision=True).pack()
        rsp = self._exec_command(Opcode.START_SWEEPER, params, bulk_data)
        return rsp.params[0]

    def _read_sweep_data(self, size):
        return self._read_rsp(size)

    def _get_sweep_fit(self, temp_hz, yield_Y, theta_deg):
        flags = 0
        if yield_Y:
            flags |= (1 << 0)

        theta_deg = theta_deg % 360
        cordic_rot = theta_deg * 2**32 // 360
        if theta_deg != 0:
            assert self.fw_version >= 0x108

        t0 = time.time_ns()
        rsp = self._exec_command(Opcode.FIT_POINTS,
                                 FitCommandPayload(flags=flags,
                                                   cordic_rot=cordic_rot,
                                                   temp_hz=temp_hz).pack(),
                                 timeout=20000, cls=FitPointsResponse)
        t1 = time.time_ns()

        return SweepFit(rsp, t1 - t0, temp_hz)

    def _start_fixed_out(self, sigout, bias):
        return self._exec_command(
            Opcode.START_FIXED_OUT,
            StartCalPayload(sigout=sigout, bias=bias).pack())

    def _set_t_enable(self, enabled):
        params = SetTEnablePayload(enabled=enabled).pack()
        self._exec_command(Opcode.SET_T_ENABLE, params)

    def _read_temp(self):
        rsp = self._exec_command(Opcode.READ_TEMP, b'', cls=ReadTempResponse)
        return rsp.osc_ticks, rsp.cpu_ticks

    def _eval_freqs(self, temp_hz, center_hz, width_hz):
        params = EvalFreqsPayload(temp_hz=temp_hz, center_hz=center_hz,
                                  width_hz=width_hz).pack()
        return self._exec_command(Opcode.EVAL_FREQS, params,
                                  cls=EvalFreqsResponse)

    def _gen_hires_freqs(self, f0, width, N):
        params = GenHiresFreqsPayload(f0=f0, width=width, N=N).pack()
        self._send_command(Opcode.GEN_HIRES_FREQS, params, b'', 1000)
        return self._read_rsp(8*(2*N + 1), timeout=1000)
