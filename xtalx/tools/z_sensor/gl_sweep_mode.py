#!/usr/bin/env python3
# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import threading
import cmath
import math

import glotlib
import numpy as np

import xtalx.z_sensor


class SweepWindow(glotlib.Window):
    def __init__(self, tc, *args, dump_fpath=None, **kwargs):
        super().__init__(900, 700, msaa=4, name=tc.serial_num or '')

        self.tc          = tc
        self.data_lock   = threading.Lock()
        self.data        = []
        self.data_gen    = -1
        self.plot_gen    = -1
        self.num_samples = 0
        self.snap        = True

        self.sweeper = xtalx.z_sensor.Sweeper(tc, *args,
                                              data_callback=self.data_callback,
                                              **kwargs)

        if dump_fpath:
            print('Dumping to %s' % dump_fpath)
            self.file = open(  # pylint: disable=R1732
                    dump_fpath, 'a', encoding='utf8')
            self.file.write('sweep,time_ns,f,dt,zx_real,zx_imag,RR,amplitude\n')
            self.file.flush()
        else:
            self.file = None

        # |Zx| plot.
        self.zx_plot = self.add_plot(
            321, limits=(self.sweeper.min_f, 0, self.sweeper.max_f, 1))
        self.zx_lines = self.zx_plot.add_lines([])
        self.zx_plot.add_hline(0, color=(0.5, 0.75, 0.5))

        # Phase plot.
        self.phi_plot = self.add_plot(
            323, limits=(self.sweeper.min_f, -math.pi,
                         self.sweeper.max_f, math.pi), sharex=self.zx_plot)
        self.phi_lines = self.phi_plot.add_lines([])
        self.phi_plot.add_hline(0, color=(0.5, 0.75, 0.5))

        # R**2 plot.
        self.rr_plot = self.add_plot(
            325, limits=(self.sweeper.min_f, 0, self.sweeper.max_f, 1.01),
            sharex=self.zx_plot)
        self.rrs_lines = [self.rr_plot.add_lines([]) for _ in tc.ADC_KEYS]

        # ADCs plot.
        self.adcs_plot = self.add_plot(
            322, limits=(self.sweeper.min_f, 0, self.sweeper.max_f, 65536),
            sharex=self.zx_plot)
        self.adcs_lines = [self.adcs_plot.add_lines([]) for _ in tc.ADC_KEYS]
        self.adcs_plot.add_hline(0, color=(0.5, 0.75, 0.5))

        # Nyquist plot.
        self.nyq_plot = self.add_plot(
            (3, 2, (4, 6)), aspect=glotlib.ASPECT_SQUARE)
        self.nyq_lines = self.nyq_plot.add_lines([], point_width=3)
        self.nyq_plot.add_hline(0, color=(0.5, 0.75, 0.5))
        self.nyq_plot.add_vline(0, color=(0.5, 0.75, 0.5))

    def start(self):
        self.sweeper.start()

    def stop(self):
        self.sweeper.stop()

    def update_geometry(self, _t):
        with self.data_lock:
            if self.data_gen == self.plot_gen:
                return False

            self.plot_gen = self.data_gen
            sdr           = self.data
            snap          = self.snap

        self.tc.info('Updating plot...')

        self.num_samples += 1

        fs = np.array([r.f for r in sdr])
        zrs = np.array([r.z.real for r in sdr])

        for i, rr_lines in enumerate(self.rrs_lines):
            rr_lines.set_x_y_data(fs, [r.RR[i] for r in sdr])

        self.zx_lines.set_x_y_data(fs, [abs(r.z) for r in sdr])
        self.phi_lines.set_x_y_data(fs, [cmath.phase(r.z) for r in sdr])
        self.nyq_lines.set_x_y_data(zrs, [-r.z.imag for r in sdr])

        if snap:
            self.zx_plot.snap_bounds()
            self.nyq_plot.snap_bounds()

        for i, adc_lines in enumerate(self.adcs_lines):
            adc_lines.set_x_y_data(fs, [r.amplitude[i] for r in sdr])
        # self.adcs_lines[0].set_x_y_data(fs, [r.z.real for r in sdr])

        return True

    def data_callback(self, sweep, t0_ns, pos, sd_results, amplitude):
        with self.data_lock:
            self.data[pos:pos+len(sd_results)] = sd_results
            self.data_gen                     += 1
            self.snap = (sweep == 1 or (sweep == 2 and pos == 0))

        if self.file:
            for r in sd_results:
                self.file.write('%s,%u,%s,%.3f,%s,%s,%s,%u\n' %
                                (sweep, t0_ns, r.f, r.nbufs / 1000., r.z.real,
                                 r.z.imag, r.RR[1], amplitude))
            self.file.flush()

        self.mark_dirty()


def main(rv):
    min_time = rv.min_freq_time_secs * rv.nfreqs
    if rv.sweep_time_secs < min_time:
        raise Exception('Sweep time must be at least %s seconds for this '
                        'choice of parameters.' % min_time)

    f0 = rv.freq_0
    f1 = rv.freq_1
    track_admittance = not rv.track_impedance

    dev = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc  = xtalx.z_sensor.make(dev, verbose=rv.verbose, yield_Y=track_admittance)

    if track_admittance:
        tc.info('Tracking admittance.')
    else:
        tc.info('Tracking impedance.')

    if not rv.amplitude:
        amplitude = tc.cal_dac_amp()
        if amplitude is None:
            raise Exception("Calibration page doesn't include the DAC voltage "
                            "under which the calibration was performed, must "
                            "specify --amplitude manually.")
    elif rv.amplitude.upper().endswith('V'):
        volts = float(rv.amplitude[:-1])
        amplitude = tc.a_to_dac(volts)
        if amplitude is None:
            raise Exception("Calibration page doesn't have required voltage-"
                            "to-DAC information to use amplitudes in Volts.")
        amplitude = round(amplitude)
    else:
        amplitude = int(rv.amplitude)

    tc.info('Using amplitude of %u DAC codes.' % amplitude)
    volts = tc.dac_to_a(amplitude)
    if volts is not None:
        tc.info('Using amplitude of %f V.' % volts)

    if not 0 <= amplitude <= 2000:
        raise Exception('Amplitude not in range 0 to 2000.')

    sw  = SweepWindow(tc, amplitude, f0, f1, rv.nfreqs, rv.sweep_time_secs,
                      rv.min_freq_time_secs, log10=rv.log10,
                      dump_fpath=rv.dump_file, settle_ms=rv.settle_ms,
                      nsweeps=rv.nsweeps)
    sw.start()

    try:
        glotlib.interact()
    except KeyboardInterrupt:
        print()
    finally:
        sw.stop()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--amplitude', '-a')
    parser.add_argument('--freq-0', '-0', type=float, required=True)
    parser.add_argument('--freq-1', '-1', type=float, required=True)
    parser.add_argument('--nfreqs', '-n', type=int, default=1000)
    parser.add_argument('--nsweeps', type=int, default=0)
    parser.add_argument('--log10', action='store_true')
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--dump-file', '-f')
    parser.add_argument('--sweep-time-secs', type=float, required=True)
    parser.add_argument('--min-freq-time-secs', type=float, default=0)
    parser.add_argument('--settle-ms', type=int, default=2)
    parser.add_argument('--sensor')
    parser.add_argument('--track-impedance', action='store_true')
    rv = parser.parse_args()

    try:
        main(rv)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
