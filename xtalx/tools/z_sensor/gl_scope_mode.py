#!/usr/bin/env python3
# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import threading
import cmath
import math
import time

import numpy
import scipy.fft

import glotlib
import glfw

import xtalx.z_sensor


class ScopeWindow(glotlib.Window):
    def __init__(self, tc, a, f, hide_fits, verbose):
        super().__init__(900, 700, msaa=4)

        self.data_lock   = threading.Lock()
        self.data        = None
        self.running     = False
        self.tc         = tc
        self.frequency   = f
        self.amplitude   = a
        self.hide_fits   = hide_fits
        self.verbose     = verbose
        self.num_samples = 0

        self.fig         = None
        self.title       = None
        self.adc         = None
        self.adc_fit     = None
        self.fft         = None

        self.t_req       = False
        self.t_act       = False
        self.paused      = False
        tc.info('Disabling T oscillator.')
        tc.set_t_enable(self.t_act)

        ax = self.add_plot((8, 1, (1, 5)),
                           limits=(0, 0, self.tc.ms_samples(),
                                   self.tc.ADC_MAX))
        self.adc = [ax.add_steps() for k in self.tc.ADC_KEYS]
        if not self.hide_fits:
            self.adc_fit = [ax.add_lines() for _ in self.tc.ADC_KEYS]

        self.fft = []

        ax = self.add_plot(817, limits=(-1000, -1000,
                                        1000 + self.tc.fft_limit(),
                                        1000 + self.tc.ADC_MAX / 2))
        self.fft.append(ax.add_steps())

        ax = self.add_plot(818, sharex=ax,
                           limits=(-1000, -1000,
                                   1000 + self.tc.fft_limit(),
                                   1000 + self.tc.ADC_MAX / 2))
        self.fft.append(ax.add_steps())

    def start(self):
        self.running = True
        threading.Thread(target=self.usb_thread).start()

    def stop(self):
        self.running = False

    def update_geometry(self, _t):
        with self.data_lock:
            sd        = self.data
            self.data = None
        if sd is None:
            return False

        w = sd.w
        X = [i/10 for i in range(self.tc.ms_samples()*10)]
        for i, si in enumerate(sd.sig_info):
            self.adc[i].set_x_y_data(si.X, si.Y)

            N  = len(si.Y)
            yf = scipy.fft.fft(si.Y)[1:N//2] / N * 2
            xf = scipy.fft.fftfreq(N, 1 / si.clk)[1:N//2]
            self.fft[i].set_x_y_data(xf, numpy.abs(yf))

            if not self.hide_fits:
                A, phase = cmath.polar(si.phasor)
                Y = [si.offset + A * math.sin(x*w + phase) for x in X]
                self.adc_fit[i].set_x_y_data(X, Y)

        self.num_samples += 1

        return True

    def handle_key_press(self, key):
        if key == glfw.KEY_T:
            self.t_req = not self.t_act
        elif key == glfw.KEY_P:
            self.paused = not self.paused
            if self.paused:
                self.tc.info('Pausing.')
            else:
                self.tc.info('Unpausing.')
        else:
            super().handle_key_press(key)

    def usb_thread(self):
        total_len = len1 = 0

        self.tc.send_scope_cmd(self.frequency, self.amplitude)
        t0 = t1 = time.time()
        while self.running:
            time.sleep(0.02)
            if self.t_req != self.t_act:
                if self.t_req:
                    self.tc.info('Enabling T oscillator.')
                else:
                    self.tc.info('Disabling T oscillator.')
                self.tc.set_t_enable(self.t_req)
                self.t_act = self.t_req

            if self.paused:
                time.sleep(0.1)
                continue

            sd = self.tc.sample_scope_sync()

            new_len = total_len + len(sd.data)
            if total_len // (1024*1024) != new_len // (1024*1024):
                t    = time.time()
                dt0  = t - t0
                dt1  = t - t1
                dlen = new_len - len1
                t1   = t
                len1 = new_len
                self.tc.info(
                    '%u bytes in %s seconds (%f bytes/s, avg %f bytes/s) '
                    '[%u / %u / %u] [%u - %u] [%u - %u]' %
                    (new_len, dt0, dlen / dt1, new_len / dt0,
                     sd.hdr.isr_cycles[0], sd.hdr.isr_cycles[1],
                     sd.hdr.isr_cycles[2],
                     min(sd.sig_info[0].Y), max(sd.sig_info[0].Y),
                     min(sd.sig_info[1].Y), max(sd.sig_info[1].Y)))
            total_len = new_len

            with self.data_lock:
                self.data = sd
                self.mark_dirty()


def main(rv):
    dev = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc  = xtalx.z_sensor.make(dev)

    tc.info('Using frequency of %s Hz.' % rv.freq)
    if rv.amplitude.upper().endswith('V'):
        volts = float(rv.amplitude[:-1])
        amplitude = round(tc.a_to_dac(volts))
        if amplitude is None:
            raise Exception("Calibration page doesn't have required voltage-"
                            "to-DAC information to use amplitudes in Volts.")

        tc.info('Using amplitude of %u = %fV' %
                (amplitude, tc.dac_to_a(amplitude)))
    else:
        amplitude = int(rv.amplitude)
        volts = tc.dac_to_a(amplitude)
        if volts is not None:
            tc.info('Using amplitude of %u = %fV' %
                    (amplitude, tc.dac_to_a(amplitude)))

    if not 0 <= amplitude <= 2000:
        raise Exception('Amplitude not in range 0 to 2000.')

    sw = ScopeWindow(tc, amplitude, rv.freq, rv.hide_fits, rv.verbose)
    sw.start()
    try:
        glotlib.interact()
    finally:
        sw.stop()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--freq', '-f', type=float, default=33000)
    parser.add_argument('--amplitude', '-a', default='0.1v')
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--hide-fits', action='store_true')
    parser.add_argument('--sensor', '-s')
    rv = parser.parse_args()

    try:
        main(rv)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
