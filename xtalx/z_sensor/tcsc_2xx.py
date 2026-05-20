# Copyright (c) 2023-2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time
import struct

import usb.core
import btype

import xtalx.usbcmd
from .tcsc_types import (DriveType, ResetReason, Opcode, GetInfoResponse,
                         SweepFit)


class GetInfoResponseStruct(btype.Struct, endian='<'):
    mcu_frequency                   = btype.uint32_t()
    aclk_divisor                    = btype.uint32_t()
    dclk_divisor                    = btype.uint32_t()
    f_hs_mhz                        = btype.uint16_t()
    cmd_buf_len                     = btype.uint16_t()
    num_usb_resets                  = btype.uint32_t()
    dv_nominal_hz                   = btype.uint32_t()
    cal_params                      = btype.uint16_t()
    reset_reason                    = btype.uint8_t()
    drive_type                      = btype.uint8_t()
    reset_csr                       = btype.uint32_t()
    reset_sr1                       = btype.uint32_t()
    max_sweep_entries               = btype.uint16_t()
    cal_dac_amplitude               = btype.uint16_t()
    electronics_calibration_date    = btype.uint32_t()
    dv_calibration_date             = btype.uint32_t()
    air_f0_milli_hz                 = btype.uint32_t()
    air_fwhm_milli_hz               = btype.uint32_t()
    _EXPECTED_SIZE                  = 56


class GetEInfoResponseStruct(btype.Struct, endian='<'):
    calibration_date                = btype.uint32_t()
    rsrv                            = btype.uint32_t()
    r_source                        = btype.float64_t()
    r_feedback                      = btype.float64_t()
    dac_to_v_coefs                  = btype.Array(btype.float64_t(), 2)
    adc_to_v_coefs                  = btype.Array(btype.float64_t(), 2)
    _EXPECTED_SIZE                  = 56


class FitPointsResponseStruct(btype.Struct, endian='<'):
    fit_status                      = btype.uint32_t()
    fit_flags                       = btype.uint32_t()
    fit_niter                       = btype.uint32_t()
    temp_c                          = btype.float32_t()
    fit_RR                          = btype.float32_t()
    strength                        = btype.float32_t()
    density_g_per_ml                = btype.float64_t()
    viscosity_cp                    = btype.float64_t()
    peak_hz                         = btype.float64_t()
    peak_fwhm                       = btype.float64_t()
    _EXPECTED_SIZE                  = 56


class EvalFreqsResponseStruct(btype.Struct, endian='<'):
    flags                           = btype.uint32_t()
    rsrv                            = btype.uint32_t()
    temp_c                          = btype.float64_t()
    density_g_per_ml                = btype.float64_t()
    viscosity_cp                    = btype.float64_t()
    _EXPECTED_SIZE                  = 32


class Comms(xtalx.usbcmd.Device):
    CMD_EP   = 0x01
    RSP_EP   = 0x81
    SCOPE_EP = 0x83

    def __init__(self, usb_dev):
        xtalx.usbcmd.Device.__init__(self, usb_dev, self.CMD_EP, self.RSP_EP,
                                     8192, 65536, git_sha1_index=4,
                                     default_configuration=0x15)

        assert self.fw_version >= 0x200

    def _read_scope(self, size, **kwargs):
        return self.usb_dev.read(self.SCOPE_EP, size, **kwargs)

    def _synchronize(self):
        xtalx.usbcmd.Device._synchronize(self)

        try:
            while True:
                self._read_scope(64, timeout=100)
        except usb.core.USBTimeoutError:
            pass

        return 0

    def _get_info(self):
        _, data = self._exec_command(Opcode.GET_INFO)
        girs = GetInfoResponseStruct.unpack(data)
        return GetInfoResponse(
                girs.mcu_frequency,
                girs.dclk_divisor,
                girs.aclk_divisor,
                girs.cmd_buf_len,
                girs.f_hs_mhz,
                DriveType(girs.drive_type),
                ResetReason(girs.reset_reason),
                girs.cal_params,
                girs.reset_csr,
                girs.reset_sr1,
                girs.max_sweep_entries,
                girs.cal_dac_amplitude,
                girs.electronics_calibration_date,
                girs.dv_calibration_date,
                girs.air_f0_milli_hz / 1000,
                girs.air_fwhm_milli_hz / 1000,
                girs.num_usb_resets,
                girs.dv_nominal_hz)

    def _get_einfo(self):
        _, data = self._exec_command(Opcode.GET_EINFO)
        if not data:
            return None
        return GetEInfoResponseStruct.unpack(data)

    def _send_scope_cmd(self, dds_skip, amplitude):
        self._exec_command(Opcode.START_SCOPER, [dds_skip, amplitude])

    def _send_auto_chirp_cmd(self, skip0, skip1, amplitude):
        self._exec_command(Opcode.AUTO_CHIRP, [skip0, skip1, amplitude])

    def _sweep_async(self, amplitude, freq_tuples, bulk_data):
        rsp, _ = self._exec_command(
                Opcode.START_SWEEPER,
                [len(freq_tuples), amplitude, True],
                bulk_data)
        return rsp.params[0]

    def _read_sweep_data(self, _size):
        while True:
            _rsp, data = self._exec_command(Opcode.READ_SWEEP_RESULT)
            if data:
                return data

    def _get_sweep_fit(self, temp_hz, yield_Y, theta_deg):
        flags = 0
        if yield_Y:
            flags |= (1 << 0)

        theta_deg = theta_deg % 360
        cordic_rot = theta_deg * 2**32 // 360

        rf_u = struct.unpack('<2I', struct.pack('<d', 0.0))
        temp_hz_u = struct.unpack('<2I', struct.pack('<d', temp_hz))

        t0 = time.time_ns()
        _rsp, data = self._exec_command(
                Opcode.FIT_POINTS,
                [flags, cordic_rot, *rf_u, *temp_hz_u],
                timeout_ms=20000)
        t1 = time.time_ns()

        fprs = FitPointsResponseStruct.unpack(data)
        return SweepFit(fprs, t1 - t0, temp_hz)

    def _start_fixed_out(self, sigout, bias):
        return self._exec_command(Opcode.START_FIXED_OUT, [sigout, bias])

    def _set_t_enable(self, enabled):
        self._exec_command(Opcode.SET_T_ENABLE, [enabled])

    def get_temp_freq(self, _cpu_freq):
        rsp, _ = self._exec_command(Opcode.READ_TEMP_FREQ)
        return struct.unpack(
                '<d', struct.pack('<2I', rsp.params[0], rsp.params[1]))[0]

    def _eval_freqs(self, temp_hz, center_hz, width_hz):
        temp_hz_u   = struct.unpack('<2I', struct.pack('<d', temp_hz))
        center_hz_u = struct.unpack('<2I', struct.pack('<d', center_hz))
        width_hz_u  = struct.unpack('<2I', struct.pack('<d', width_hz))
        _rsp, data  = self._exec_command(
                Opcode.EVAL_FREQS,
                [*temp_hz_u, *center_hz_u, *width_hz_u])
        return EvalFreqsResponseStruct.unpack(data)

    def _gen_hires_freqs(self, f0, width, N):
        f0_u    = struct.unpack('<2I', struct.pack('<d', f0))
        width_u = struct.unpack('<2I', struct.pack('<d', width))
        _rsp, data = self._exec_command(
                Opcode.GEN_HIRES_FREQS,
                [*f0_u, *width_u, N])
        return data
