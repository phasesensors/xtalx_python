# Copyright (c) 2023-2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import math
import time
import struct

import usb.core
import btype
import numpy as np

import xtalx.usbcmd
from .tincan import TinCan
from .scope_data import ScopeData
from .tcsc_types import (DriveType, ResetReason, Opcode, GetInfoResponse,
                         SampleHeader, AutoChirpHeader, AutoChirpResult,
                         SweepEntry, SweepResult, SweepFit, ParsedSweepResult)
from . import crystal_info


class GetInfoResponseStruct(btype.Struct):
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


class GetEInfoResponseStruct(btype.Struct):
    calibration_date                = btype.uint32_t()
    r_source                        = btype.float64_t()
    r_feedback                      = btype.float64_t()
    dac_to_v_coefs                  = btype.Array(btype.float64_t(), 2)
    adc_to_v_coefs                  = btype.Array(btype.float64_t(), 2)
    _EXPECTED_SIZE                  = 56


class FitPointsResponseStruct(btype.Struct):
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


class EvalFreqsResponseStruct(btype.Struct):
    flags                           = btype.uint32_t()
    rsrv                            = btype.uint32_t()
    temp_c                          = btype.float64_t()
    density_g_per_ml                = btype.float64_t()
    viscosity_cp                    = btype.float64_t()
    _EXPECTED_SIZE                  = 32


class TCSC(TinCan,
           xtalx.usbcmd.Device):
    CMD_EP   = 0x01
    RSP_EP   = 0x81
    SCOPE_EP = 0x83

    DAC_MAX  = None
    ADC_MAX  = None
    ADC_KEYS = ()

    def __init__(self, usb_dev, **kwargs):
        TinCan.__init__(self, **kwargs)
        xtalx.usbcmd.Device.__init__(self, usb_dev, self.CMD_EP, self.RSP_EP,
                                     8192, 65536, git_sha1_index=4,
                                     default_configuration=0x15)

        self.info('Controlling sensor %s with firmware 0x%X (%s).' %
                  (self.serial_num, self.fw_version, self.git_sha1))

        assert self.fw_version >= 0x200

        self.ginfo    = self._get_info()
        self.einfo    = self._get_einfo()
        self.CPU_FREQ = self.ginfo.hclk

        self.crystal_info = crystal_info.CRYSTAL_INFOS.get(
                self.ginfo.dv_nominal_hz)

        self._sweep_params = None

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
        return GetEInfoResponseStruct.unpack(data)

    def fft_limit(self):
        return self.ginfo.aclk / 4

    def ms_samples(self):
        return int(self.ginfo.aclk / 1000)

    def cal_dac_amp(self):
        return self.ginfo.cal_dac_amplitude

    def v_to_adc(self, v):
        '''
        Return the ADC value that would be measured for the given voltage.

        Note that ADC values are signed because the TCSC FW shifts the values
        down by ADC_MAX / 2 to center on the origin for stability in the
        floating-point math.
        '''
        if not self.ginfo.electronics_cal_date:
            return None
        return ((v - self.einfo.adc_to_v_coefs[0]) /
                self.einfo.adc_to_v_coefs[1])

    def adc_to_v(self, adc):
        '''
        Return the voltage that would generate the given ADC value.

        Note that ADC values are signed because the TCSC FW shifts the values
        down by ADC_MAX / 2 to center on the origin for stability in the
        floating-point math.
        '''
        if not self.ginfo.electronics_cal_date:
            return None
        return (self.einfo.adc_to_v_coefs[0] +
                adc * self.einfo.adc_to_v_coefs[1])

    def v_to_dac(self, v):
        '''
        Returns the DAC value corresponding with the specified output voltage
        if the bias voltage is set to half VREF (i.e. 2048).  Otherwise, this
        value is only suitable for relative voltage calculations.
        '''
        if not self.ginfo.electronics_cal_date:
            return None
        return ((v - self.einfo.dac_to_v_coefs[0]) /
                self.einfo.dac_to_v_coefs[1])

    def a_to_dac(self, a):
        '''
        Given an amplitude in volts, return the equivalent amplitude in DAC
        units.  Note that an amplitude is a relative voltage measurement and
        only takes into account the gain; the offset will be around the bias
        voltage but is not calibrated.
        '''
        if not self.ginfo.electronics_cal_date:
            return None
        return -a / self.einfo.dac_to_v_coefs[1]

    def dac_to_a(self, dac):
        '''
        Given an amplitude in DAC units, return the equivalent amplitude in
        volts.  Note that an amplitude is a relative voltage measurement and
        only takes into account the gain; the offset will be around the bias
        voltage but is not calibrated.
        '''
        if not self.ginfo.electronics_cal_date:
            return None
        return -dac * self.einfo.dac_to_v_coefs[1]

    def parse_amplitude(self, amplitude):
        if amplitude is None:
            amplitude = self.cal_dac_amp()
            if amplitude is None:
                raise Exception("Calibration page doesn't include the DAC "
                                "voltage under which the calibration was "
                                "performed, must specify --amplitude manually.")
            return amplitude

        if amplitude.upper().endswith('V'):
            volts = float(amplitude[:-1])
            amplitude = self.a_to_dac(volts)
            if amplitude is None:
                raise Exception("Calibration page doesn't have required "
                                "voltage-to-DAC information to use amplitudes "
                                "in Volts.")
            return round(amplitude)

        return int(amplitude)

    def _read_samples(self):
        data    = self._read_scope(768 * 1024, timeout=3000)
        hdr     = SampleHeader.unpack(data[:SampleHeader._STRUCT.size])
        assert len(data) == SampleHeader._STRUCT.size + hdr.nsamples * 2
        samples = np.frombuffer(data, dtype='<i2',
                                offset=SampleHeader._STRUCT.size)
        assert len(samples) == hdr.nsamples

        w    = 2 * math.pi * hdr.adc_skip / 2**32
        freq = self.ginfo.dclk * hdr.dds_skip / 2**32
        sd   = ScopeData(freq, w, hdr=hdr, data=data)
        for i, name in enumerate(self.ADC_KEYS):
            offset = hdr.sfp[i].offset + self.ADC_MAX / 2
            X      = np.arange(i, len(samples), 2)
            Y      = samples[i::2] + self.ADC_MAX / 2
            clk    = self.ginfo.aclk / 2
            sd.add_signal(X, Y, clk, name, offset=offset)

        return sd, hdr

    def send_scope_cmd(self, frequency, amplitude):
        dds_skip = int(round(2**32 * (frequency / self.ginfo.dclk)))
        rsp, _ = self._exec_command(Opcode.START_SCOPER, [dds_skip, amplitude])
        return rsp

    def sample_scope_sync(self, **_kwargs):
        sd, hdr = self._read_samples()

        for si, sfp in zip(sd.sig_info, hdr.sfp):
            si.phasor = sfp.x + sfp.y * 1j
            si.RR     = sfp.RR

        return sd

    def send_auto_chirp_cmd(self, f0, f1, amplitude):
        skip0 = int(round(2**32 * (f0 / self.ginfo.dclk)))
        skip1 = int(round(2**32 * (f1 / self.ginfo.dclk)))
        return self._exec_command(Opcode.AUTO_CHIRP, [skip0, skip1, amplitude])

    def sample_auto_chirp_sync(self):
        data = self._read_scope(128*1024, timeout=3000)
        assert len(data) == 16432

        hdr  = AutoChirpHeader.unpack(data[:AutoChirpHeader._STRUCT.size])
        bins = np.frombuffer(data, dtype='<f',
                             offset=AutoChirpHeader._STRUCT.size)
        return AutoChirpResult(hdr, bins)

    def sweep_async(self, amplitude, freq_tuples, ndiscards=2):
        assert 2 <= len(freq_tuples) <= self.ginfo.max_sweep_entries

        self._sweep_params = []
        bulk_data = b''
        for i, (f, nbufs) in enumerate(freq_tuples):
            dds_skip = int(round(2**32 * (f / self.ginfo.dclk)))
            freq     = self.ginfo.dclk * dds_skip / 2**32
            bulk_data += SweepEntry(dds_skip=dds_skip,
                                    ndiscards=ndiscards if i == 0 else 5,
                                    nbufs=nbufs).pack()
            self._sweep_params.append((freq, nbufs))

        rsp, _ = self._exec_command(
                Opcode.START_SWEEPER,
                [len(freq_tuples), amplitude, True],
                bulk_data)
        sleep_ms = rsp.params[0]
        wakeup_time = time.localtime(time.time() + sleep_ms / 1000)

        self.info('Sweeping %u frequencies from %.3f-%.3f Hz.  Sensor '
                  'recommends sleep of %u ms until %s.' %
                  (len(freq_tuples), freq_tuples[0][0], freq_tuples[-1][0],
                   sleep_ms, time.asctime(wakeup_time)))

        return sleep_ms / 1000, sleep_ms

    def read_sweep_data(self, theta_deg=0):
        '''
        Returns an array of ParsedSweepResult objects containing the results of
        the sweep.  Each element is the result of the corresponding element in
        the _sweep_params array that was built in sweep_async().
        '''
        while True:
            _rsp, data = self._exec_command(Opcode.READ_SWEEP_RESULT)
            if data:
                break

        size = SweepResult._STRUCT.size * len(self._sweep_params)
        assert len(data) == size

        theta_deg = theta_deg % 360
        theta_rad = theta_deg * math.pi / 180

        results = []
        offset  = 0
        for freq, nbufs in self._sweep_params:
            sr = SweepResult.unpack_from(data, offset=offset)
            sr.offset[0] += self.ADC_MAX / 2
            sr.offset[1] += self.ADC_MAX / 2
            results.append(ParsedSweepResult(sr, self.einfo.r_feedback,
                                             freq, nbufs, self.yield_Y,
                                             theta_rad))
            offset += SweepResult._STRUCT.size

        return results

    def get_sweep_fit(self, temp_hz, theta_deg=0):
        flags = 0
        if self.yield_Y:
            flags |= (1 << 0)

        theta_deg = theta_deg % 360
        cordic_rot = theta_deg * 2**32 // 360
        if theta_deg != 0:
            assert self.fw_version >= 0x108

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

    def start_fixed_out(self, sigout, bias):
        return self._exec_command(Opcode.START_FIXED_OUT, [sigout, bias])

    def set_t_enable(self, enabled):
        self._exec_command(Opcode.SET_T_ENABLE, [enabled])

    def read_temp(self):
        rsp, _ = self._exec_command(Opcode.READ_TEMP)
        cpu_ticks = (rsp.params[0] | (rsp.params[1] << 32))
        osc_ticks = rsp.params[2]
        return osc_ticks, cpu_ticks

    def eval_freqs(self, temp_hz, center_hz, width_hz):
        temp_hz_u   = struct.unpack('<2I', struct.pack('<d', temp_hz))
        center_hz_u = struct.unpack('<2I', struct.pack('<d', center_hz))
        width_hz_u  = struct.unpack('<2I', struct.pack('<d', width_hz))
        _rsp, data  = self._exec_command(
                Opcode.EVAL_FREQS,
                [*temp_hz_u, *center_hz_u, *width_hz_u])

        efrs = EvalFreqsResponseStruct.unpack(data)
        temp_c = efrs.temp_c if efrs.flags & 1 else None
        if center_hz and width_hz:
            density_g_per_ml = efrs.density_g_per_ml if efrs.flags & 2 else None
            viscosity_cp     = efrs.viscosity_cp if efrs.flags & 4 else None
        else:
            density_g_per_ml = None
            viscosity_cp     = None
        return (temp_c, density_g_per_ml, viscosity_cp)

    def gen_hires_freqs(self, f0, width, N):
        '''
        Generate a list of hi-resolution frequencies to be measured, centered
        on the peak at f0 of the specified width, with N specifying the number
        of frequencies in each wing.  A total of 2N + 1 frequencies will be
        returned.
        '''
        f0_u    = struct.unpack('<2I', struct.pack('<d', f0))
        width_u = struct.unpack('<2I', struct.pack('<d', width))
        _rsp, data = self._exec_command(
                Opcode.GEN_HIRES_FREQS,
                [*f0_u, *width_u, N])

        N    = 2*N + 1
        size = 8*N
        assert len(data) == size
        return struct.unpack('<%ud' % N, data)
