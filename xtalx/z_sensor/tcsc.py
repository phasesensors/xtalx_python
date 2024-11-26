# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import random
import cmath
import math
import time
import struct
from enum import IntEnum

import usb.core
import btype
import numpy as np

from .tincan import TinCan
from .scope_data import ScopeData
from . import crystal_info


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


class Opcode(IntEnum):
    GET_INFO        = 1
    START_SCOPER    = 2
    START_SWEEPER   = 4
    START_FIXED_OUT = 6
    SET_T_ENABLE    = 8
    READ_TEMP       = 9
    FIT_POINTS      = 10
    EVAL_FREQS      = 14
    GEN_HIRES_FREQS = 15
    GET_EINFO       = 16
    AUTO_CHIRP      = 17
    BAD_OPCODE      = 0xCCCC


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


class GetInfoResponse:
    def __init__(self, rsp, tc):
        self.hclk                 = rsp.params[0]
        self.dclk_divisor         = rsp.params[1] >> 16
        self.aclk_divisor         = rsp.params[1] & 0xFFFF
        self.cmd_buf_len          = rsp.params[2] >> 10
        self.f_hs_mhz             = rsp.params[2] & 0x03FF
        self.dclk                 = self.hclk / self.dclk_divisor
        self.aclk                 = self.hclk / self.aclk_divisor
        self.drive_type           = DriveType(rsp.params[4] & 0xFF)
        self.reset_reason         = ResetReason((rsp.params[4] >> 8) & 0xFF)
        self.cal_params           = rsp.params[4] >> 16
        self.reset_csr            = rsp.params[5]
        self.reset_sr1            = rsp.params[6]
        self.max_sweep_entries    = rsp.params[7] & 0xFFFF
        self.cal_dac_amplitude    = rsp.params[7] >> 16
        self.electronics_cal_date = rsp.params[8]
        self.crystal_cal_date     = rsp.params[9]
        self.air_f0               = rsp.params[10] / 1000
        self.air_fwhm             = rsp.params[11] / 1000

        if tc.fw_version < 0x106:
            self.nresets       = rsp.params[3]
            self.dv_nominal_hz = 32768
        else:
            self.nresets       = (rsp.params[3] >> 24) & 0xFF
            self.dv_nominal_hz = (rsp.params[3] & 0xFFFFFF) or 32768

    def have_temp_cal(self):
        return self.cal_params & (1 << 0)

    def get_drive_type(self):
        return DRIVE_TYPE_MAP.get(self.drive_type, 'Really Unknown')


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


class StartSweepPayload(btype.Struct):
    nfreqs              = btype.uint32_t()
    amplitude           = btype.uint32_t()
    double_precision    = btype.uint32_t()
    _EXPECTED_SIZE      = 12


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


class SweepData:
    def __init__(self, sweep_results, sweep_params, rf, yield_Y, theta_deg):
        self.results = []
        theta_deg = theta_deg % 360
        theta_rad = theta_deg * math.pi / 180
        for sr, (freq, nbufs) in zip(sweep_results, sweep_params):
            self.results.append(ParsedSweepResult(sr, rf, freq, nbufs, yield_Y,
                                                  theta_rad))


class TCSC(TinCan):
    CMD_EP   = 0x01
    RSP_EP   = 0x82
    SCOPE_EP = 0x83

    DAC_MAX  = None
    ADC_MAX  = None
    ADC_KEYS = ()

    def __init__(self, usb_dev, **kwargs):
        super().__init__(**kwargs)

        self.usb_dev = usb_dev

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 4)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise Exception(
                    'Device has no langid, ensure running as root!') from e
        self.info('Controlling sensor %s with firmware 0x%X (%s).' %
                  (self.serial_num, self.fw_version, self.git_sha1))

        if self.fw_version < 0x101:
            raise Exception('Firmware version of 0x%X not supported.' %
                            self.fw_version)

        self.tag = random.randint(1, 0xFFFF)

        self._set_configuration(usb_dev, 0x15)
        self._synchronize()

        self.ginfo    = self._get_info()
        self.einfo    = self._get_einfo()
        self.CPU_FREQ = self.ginfo.hclk

        self.crystal_info = crystal_info.CRYSTAL_INFOS.get(
                self.ginfo.dv_nominal_hz)

        self._sweep_params = None

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

    def _send_abort(self):
        self.usb_dev.write(self.CMD_EP, b'')

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
            self.usb_dev.write(self.CMD_EP, data)
            rsp = self.usb_dev.read(self.RSP_EP, 64, timeout=100)
            if len(rsp) == 56:
                break

            junk += len(rsp)

        self.usb_dev.write(self.CMD_EP, data)
        data = self.usb_dev.read(self.RSP_EP, 64)
        assert len(data) == 56

        rsp = Response.unpack(data)
        assert rsp.tag    == tag
        assert rsp.opcode == Opcode.BAD_OPCODE
        assert rsp.status == Status.BAD_OPCODE

        junk_len = 0
        try:
            while True:
                junk_len += len(self.usb_dev.read(self.SCOPE_EP, 64,
                                                  timeout=100))
        except usb.core.USBTimeoutError:
            pass

        return junk

    def _send_command(self, opcode, params, bulk_data, timeout):
        tag  = self._alloc_tag()
        hdr  = CommandHeader(opcode=opcode, tag=tag)
        data = hdr.pack() + params + bytes(48 - len(params)) + bulk_data
        size = self.usb_dev.write(self.CMD_EP, data, timeout=timeout)
        assert size == len(data)
        if (len(data) % 64) == 0:
            if len(data) < self.ginfo.cmd_buf_len:
                self.usb_dev.write(self.CMD_EP, b'')
        return tag

    def _recv_response(self, tag, timeout, cls=Response):
        data = self.usb_dev.read(self.RSP_EP, Response._STRUCT.size,
                                 timeout=timeout)
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
        return GetInfoResponse(self._exec_command(Opcode.GET_INFO), self)

    def _get_einfo(self):
        return self._exec_command(Opcode.GET_EINFO, cls=GetEInfoResponse)

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
        data    = self.usb_dev.read(self.SCOPE_EP, 768 * 1024, timeout=3000)
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
        return self._exec_command(
            Opcode.START_SCOPER,
            StartScoperPayload(dds_skip=dds_skip, amplitude=amplitude).pack())

    def sample_scope_sync(self, **_kwargs):
        sd, hdr = self._read_samples()

        for si, sfp in zip(sd.sig_info, hdr.sfp):
            si.phasor = sfp.x + sfp.y * 1j
            si.RR     = sfp.RR

        return sd

    def send_auto_chirp_cmd(self, f0, f1, amplitude):
        skip0 = int(round(2**32 * (f0 / self.ginfo.dclk)))
        skip1 = int(round(2**32 * (f1 / self.ginfo.dclk)))
        return self._exec_command(
            Opcode.AUTO_CHIRP,
            AutoChirpPayload(skip0=skip0, skip1=skip1,
                             amplitude=amplitude).pack())

    def sample_auto_chirp_sync(self):
        data = self.usb_dev.read(self.SCOPE_EP, 128*1024, timeout=3000)
        assert len(data) == 16432

        hdr  = AutoChirpHeader.unpack(data[:AutoChirpHeader._STRUCT.size])
        bins = np.frombuffer(data, dtype='<f',
                             offset=AutoChirpHeader._STRUCT.size)
        return AutoChirpResult(hdr, bins)

    def sweep_async(self, amplitude, freq_tuples, ndiscards=2):
        assert 2 <= len(freq_tuples) <= self.ginfo.max_sweep_entries

        self._sweep_params = []
        params = StartSweepPayload(nfreqs=len(freq_tuples),
                                   amplitude=amplitude,
                                   double_precision=True).pack()
        bulk_data = b''
        for i, (f, nbufs) in enumerate(freq_tuples):
            dds_skip = int(round(2**32 * (f / self.ginfo.dclk)))
            freq     = self.ginfo.dclk * dds_skip / 2**32
            bulk_data += SweepEntry(dds_skip=dds_skip,
                                    ndiscards=ndiscards if i == 0 else 5,
                                    nbufs=nbufs).pack()
            self._sweep_params.append((freq, nbufs))

        rsp = self._exec_command(Opcode.START_SWEEPER, params, bulk_data)
        sleep_ms = rsp.params[0]
        wakeup_time = time.localtime(time.time() + sleep_ms / 1000)

        self.info('Sweeping %u frequencies from %.3f-%.3f Hz.  Sensor '
                  'recommends sleep of %u ms until %s.' %
                  (len(freq_tuples), freq_tuples[0][0], freq_tuples[-1][0],
                   sleep_ms, time.asctime(wakeup_time)))

        return sleep_ms / 1000, sleep_ms

    def read_sweep_data(self, theta_deg=0):
        size = SweepResult._STRUCT.size * len(self._sweep_params)
        data = self.usb_dev.read(self.RSP_EP, size)
        assert len(data) == size

        results = []
        while data:
            sr = SweepResult.unpack(data[:SweepResult._STRUCT.size])
            sr.offset[0] += self.ADC_MAX / 2
            sr.offset[1] += self.ADC_MAX / 2
            results.append(sr)
            data = data[SweepResult._STRUCT.size:]

        return SweepData(results, self._sweep_params, self.einfo.r_feedback,
                         self.yield_Y, theta_deg)

    def get_sweep_fit(self, temp_hz, theta_deg=0):
        flags = 0
        if self.yield_Y:
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

    def start_fixed_out(self, sigout, bias):
        '''
        Generate a fixed voltage output, setting the output op-amp's positive
        input to bias and negative input to sigout.  This is typically used
        for calibration purposes, but setting sigout=0 and bias=0 can also be
        used to quiesece the sensor by grounding the output.
        '''
        return self._exec_command(
            Opcode.START_FIXED_OUT,
            StartCalPayload(sigout=sigout, bias=bias).pack())

    def set_t_enable(self, enabled):
        '''
        Enable or disable the temperature oscillators to reduce noise on the
        density/viscosity measurement.
        '''
        params = SetTEnablePayload(enabled=enabled).pack()
        self._exec_command(Opcode.SET_T_ENABLE, params)

    def read_temp(self):
        rsp = self._exec_command(Opcode.READ_TEMP, b'', cls=ReadTempResponse)
        return rsp.osc_ticks, rsp.cpu_ticks

    def eval_freqs(self, temp_hz, center_hz, width_hz):
        '''
        Get the sensor to manually convert a set of frequencies into
        temperature, density and viscosity values.  To only convert
        temperature, set center_hz=0 and width_hz=0.
        '''
        params = EvalFreqsPayload(temp_hz=temp_hz, center_hz=center_hz,
                                  width_hz=width_hz).pack()
        rsp    = self._exec_command(Opcode.EVAL_FREQS, params,
                                    cls=EvalFreqsResponse)
        temp_c = rsp.temp_c if rsp.flags & 1 else None
        if center_hz and width_hz:
            density_g_per_ml = rsp.density_g_per_ml if rsp.flags & 2 else None
            viscosity_cp     = rsp.viscosity_cp if rsp.flags & 4 else None
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
        params = GenHiresFreqsPayload(f0=f0, width=width, N=N).pack()
        self._send_command(Opcode.GEN_HIRES_FREQS, params, b'', 1000)

        N    = 2*N + 1
        size = 8*N
        data = self.usb_dev.read(self.RSP_EP, size, timeout=1000)
        return struct.unpack('<%ud' % N, data)
