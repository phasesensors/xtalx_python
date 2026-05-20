# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time
import math
import struct

import numpy as np

from .tincan import TinCan
from .scope_data import ScopeData
from .tcsc_types import (SampleHeader, AutoChirpHeader, AutoChirpResult,
                         SweepEntry, SweepResult, ParsedSweepResult)
from . import crystal_info


class TCSC_U5(TinCan):
    DAC_MAX  = 4096
    ADC_MAX  = 16384
    ADC_KEYS = ('PROBEA', 'SIGIN')

    def __init__(self, comms, **kwargs):
        super().__init__(**kwargs)

        self.comms = comms

        self.serial_num = comms.serial_num
        self.git_sha1   = comms.git_sha1
        self.fw_version = comms.fw_version

        self.info('Controlling sensor %s with firmware 0x%X (%s).' %
                  (self.serial_num, self.fw_version, self.git_sha1))

        self.ginfo    = self.comms._get_info()
        self.einfo    = self.comms._get_einfo()
        self.CPU_FREQ = self.ginfo.hclk

        self.crystal_info = crystal_info.CRYSTAL_INFOS.get(
                self.ginfo.dv_nominal_hz)

        self._sweep_params = None

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
        data    = self.comms._read_scope(768 * 1024, timeout=3000)
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
        self.comms._send_scope_cmd(dds_skip, amplitude)

    def sample_scope_sync(self, **_kwargs):
        sd, hdr = self._read_samples()

        for si, sfp in zip(sd.sig_info, hdr.sfp):
            si.phasor = sfp.x + sfp.y * 1j
            si.RR     = sfp.RR

        return sd

    def send_auto_chirp_cmd(self, f0, f1, amplitude):
        skip0 = int(round(2**32 * (f0 / self.ginfo.dclk)))
        skip1 = int(round(2**32 * (f1 / self.ginfo.dclk)))
        self.comms._send_auto_chirp_cmd(skip0, skip1, amplitude)

    def sample_auto_chirp_sync(self):
        data = self.comms._read_scope(128*1024, timeout=3000)
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

        sleep_ms = self.comms._sweep_async(amplitude, freq_tuples, bulk_data)

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
        size = SweepResult._STRUCT.size * len(self._sweep_params)
        data = self.comms._read_sweep_data(size)
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
        return self.comms._get_sweep_fit(temp_hz, self.yield_Y, theta_deg)

    def start_fixed_out(self, sigout, bias):
        '''
        Generate a fixed voltage output, setting the output op-amp's positive
        input to bias and negative input to sigout.  This is typically used
        for calibration purposes, but setting sigout=0 and bias=0 can also be
        used to quiesece the sensor by grounding the output.
        '''
        self.comms._start_fixed_out(sigout, bias)

    def set_t_enable(self, enabled):
        '''
        Enable or disable the temperature oscillators to reduce noise on the
        density/viscosity measurement.
        '''
        self.comms._set_t_enable(enabled)

    def get_temp_freq(self):
        return self.comms.get_temp_freq(self.CPU_FREQ)

    def eval_freqs(self, temp_hz, center_hz, width_hz):
        '''
        Get the sensor to manually convert a set of frequencies into
        temperature, density and viscosity values.  To only convert
        temperature, set center_hz=0 and width_hz=0.
        '''
        rsp = self.comms._eval_freqs(temp_hz, center_hz, width_hz)
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
        data = self.comms._gen_hires_freqs(f0, width, N)
        N    = 2*N + 1
        size = 8*N
        assert len(data) == size
        return struct.unpack('<%ud' % N, data)
