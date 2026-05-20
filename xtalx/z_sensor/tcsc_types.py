# Copyright (c) 2023-2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import cmath
import math
from enum import IntEnum

import btype


class DriveType(IntEnum):
    UNKNOWN_DRIVE  = 0
    EXTERNAL_DRIVE = 1
    INTERNAL_DRIVE = 2


DRIVE_TYPE_MAP = {
    DriveType.UNKNOWN_DRIVE  : 'Unknwon',
    DriveType.EXTERNAL_DRIVE : 'External',
    DriveType.INTERNAL_DRIVE : 'Internal',
}


class ResetReason(IntEnum):
    UNKNOWN         = 0
    POWER_ON_RESET  = 1
    SOFTWARE_RESET  = 2
    STANDBY_RESET   = 3
    NRST_PIN        = 4


class Opcode(IntEnum):
    GET_INFO            = 1
    START_SCOPER        = 2
    START_SWEEPER       = 4
    START_FIXED_OUT     = 6
    SET_T_ENABLE        = 8
    READ_TEMP_COUNTS    = 9
    FIT_POINTS          = 10
    EVAL_FREQS          = 14
    GEN_HIRES_FREQS     = 15
    GET_EINFO           = 16
    AUTO_CHIRP          = 17
    READ_SWEEP_RESULT   = 21
    READ_TEMP_FREQ      = 22
    BAD_OPCODE          = 0xCCCC


class GetInfoResponse:
    def __init__(self, hclk, dclk_divisor, aclk_divisor, cmd_buf_len, f_hs_mhz,
                 drive_type, reset_reason, cal_params, reset_csr,
                 reset_sr1, max_sweep_entries, cal_dac_amplitude,
                 electronics_cal_date, crystal_cal_date, air_f0, air_fwhm,
                 nresets, dv_nominal_hz):
        self.hclk                 = hclk
        self.dclk_divisor         = dclk_divisor
        self.aclk_divisor         = aclk_divisor
        self.cmd_buf_len          = cmd_buf_len
        self.f_hs_mhz             = f_hs_mhz
        self.dclk                 = self.hclk / self.dclk_divisor
        self.aclk                 = self.hclk / self.aclk_divisor
        self.drive_type           = drive_type
        self.reset_reason         = reset_reason
        self.cal_params           = cal_params
        self.reset_csr            = reset_csr
        self.reset_sr1            = reset_sr1
        self.max_sweep_entries    = max_sweep_entries
        self.cal_dac_amplitude    = cal_dac_amplitude
        self.electronics_cal_date = electronics_cal_date
        self.crystal_cal_date     = crystal_cal_date
        self.air_f0               = air_f0
        self.air_fwhm             = air_fwhm
        self.nresets              = nresets
        self.dv_nominal_hz        = dv_nominal_hz

    def have_temp_cal(self):
        return self.cal_params & (1 << 0)

    def get_drive_type(self):
        return DRIVE_TYPE_MAP.get(self.drive_type, 'Really Unknown')


class SineFitPhasor(btype.Struct):
    x              = btype.float64_t()
    y              = btype.float64_t()
    offset         = btype.float64_t()
    RR             = btype.float64_t()
    _EXPECTED_SIZE = 32


class SampleHeader(btype.Struct):
    sfp            = btype.Array(SineFitPhasor(), 2)
    dds_skip       = btype.uint32_t()
    adc_skip       = btype.uint32_t()
    nsamples       = btype.uint32_t()
    t_ms           = btype.uint32_t()
    tag            = btype.uint16_t()
    rsrv           = btype.uint16_t()
    isr_cycles     = btype.Array(btype.uint32_t(), 3)
    _EXPECTED_SIZE = 96


class AutoChirpHeader(btype.Struct):
    nchirps         = btype.uint32_t()
    bin0            = btype.uint32_t()
    bin1            = btype.uint32_t()
    rsrv            = btype.uint32_t()
    bin_width       = btype.float64_t()
    A               = btype.float64_t()
    x0              = btype.float64_t()
    W               = btype.float64_t()
    _EXPECTED_SIZE  = 48


class AutoChirpResult:
    def __init__(self, hdr, bins):
        self.nchirps   = hdr.nchirps
        self.bin0      = hdr.bin0
        self.bin1      = hdr.bin1
        self.nbins     = hdr.bin1 - hdr.bin0 + 1
        self.bin_width = hdr.bin_width
        self.f0        = hdr.bin0 * hdr.bin_width
        self.f1        = hdr.bin1 * hdr.bin_width
        self.A         = hdr.A
        self.x0        = hdr.x0
        self.W         = hdr.W
        self.hdr       = hdr
        self.bins      = bins / hdr.nchirps


class SweepEntry(btype.Struct):
    dds_skip        = btype.uint32_t()
    ndiscards       = btype.uint16_t()
    nbufs           = btype.uint16_t()
    _EXPECTED_SIZE  = 8


class SweepResult(btype.Struct):
    real            = btype.Array(btype.float64_t(), 2)
    imag            = btype.Array(btype.float64_t(), 2)
    offset          = btype.Array(btype.float64_t(), 2)
    RR              = btype.Array(btype.float64_t(), 2)
    _EXPECTED_SIZE  = 64


class SweepFit:
    def __init__(self, fpr, dt, temp_hz):
        self.status    = fpr.fit_status
        self.flags     = fpr.fit_flags
        self.niter     = fpr.fit_niter
        self.RR        = fpr.fit_RR
        self.peak_hz   = fpr.peak_hz
        self.peak_fwhm = fpr.peak_fwhm
        self.strength  = fpr.strength
        self.dt        = dt
        self.temp_hz   = temp_hz

        if self.flags & (1 << 0):
            self.temp_c = fpr.temp_c
        else:
            self.temp_c = None

        if self.flags & (1 << 1):
            self.density_g_per_ml = fpr.density_g_per_ml
        else:
            self.density_g_per_ml = None

        if self.flags & (1 << 2):
            self.viscosity_cp = fpr.viscosity_cp
        else:
            self.viscosity_cp = None


class ParsedSweepResult:
    def __init__(self, sweep_result, rf, freq, nbufs, yield_Y, theta_rad):
        self._sweep_result = sweep_result

        phasors = [sweep_result.real[0] + sweep_result.imag[0] * 1j,
                   sweep_result.real[1] + sweep_result.imag[1] * 1j]

        self.amplitude = [abs(p) for p in phasors]
        self.phase     = [cmath.phase(p) for p in phasors]
        self.RR        = list(sweep_result.RR)
        self.nbufs     = nbufs
        self.f         = freq

        probea = phasors[0]
        sigin  = phasors[1]
        Z      = -probea * rf / sigin
        self.Z = complex(
                Z.real * math.cos(theta_rad) - Z.imag * math.sin(theta_rad),
                Z.real * math.sin(theta_rad) + Z.imag * math.cos(theta_rad))
        self.Y = 1 / self.Z
        self.z = self.Y if yield_Y else self.Z
