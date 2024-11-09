# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import time
import cmath
import math
from enum import Enum

import glotlib

import xtalx.z_sensor
from xtalx.z_sensor.peak_tracker import Delegate

from . import z_common


def interp(v0, vN, N, t):
    return (vN*t + v0*(N-t)) / N


HISTORY_PLOTS = 5
BASE_COLOR     = (0x1F/255, 0x77/255, 0xB4/255)
HISTORY_COLORS = [(interp(BASE_COLOR[0], 1, HISTORY_PLOTS, t),
                   interp(BASE_COLOR[1], 1, HISTORY_PLOTS, t),
                   interp(BASE_COLOR[2], 1, HISTORY_PLOTS, t),
                   1)
                  for t in range(HISTORY_PLOTS)]


LINE_WIDTH = 1


class TrackMode(Enum):
    CHIRP   = 1
    SWEEP   = 2


class ViewMode(Enum):
    LARGE   = 1
    SMALL   = 2


class TrackerWindow(glotlib.Window, Delegate):
    def __init__(self, tc, ipq, z_args, z_logger, name):
        super().__init__(900, 700, msaa=4, name=tc.serial_num or '')

        self.tc               = tc
        self.ipq              = ipq
        self.z_args           = z_args
        self.z_logger         = z_logger
        self.data_gen         = -1
        self.plot_gen         = -1
        self.data_lock        = threading.Lock()
        self.sweeps           = [[] for _ in range(HISTORY_PLOTS)]
        self.f_centers        = []
        self.widths           = []
        self.strengths        = []
        self.temperatures     = []
        self.densities        = []
        self.viscosities      = []
        self.tags             = []
        self.chirp_X_data     = None
        self.chirp_Y_data     = None
        self.chirp_X_lorentz  = None
        self.chirp_Y_lorentz  = None
        self.chirp_fit        = None
        self.chirp_freq       = None
        self.chirp_width      = None
        self.chirp_strength   = None
        self.sweep_snap       = True
        self.track_mode       = None
        self.view_mode        = ViewMode.LARGE
        self.large_w          = self.w_w
        self.large_h          = self.w_h
        self.small_w          = round(self.w_w * 0.75)
        self.small_h          = round(self.w_h * 0.25)
        self.sweep_prefix     = ''
        self.end_sweep_time   = None

        volts = tc.dac_to_a(z_args.amplitude)
        if volts:
            self.drive_str = 'Drive: %.2fV' % volts
        else:
            self.drive_str = 'Drive: %u DAC' % z_args.amplitude

        self.d_plot = self.add_plot(
            511, limits=(-0.1, -0.05, 50, 1.05), max_v_ticks=4)
        self.d_lines = self.d_plot.add_lines([], width=LINE_WIDTH)

        self.v_plot = self.add_plot(
            512, limits=(-0.1, -0.5, 50, 30.5), max_v_ticks=4,
            sharex=self.d_plot)
        self.v_lines = self.v_plot.add_lines([], width=LINE_WIDTH)

        t_lim = (262000, 262600) if tc.ginfo.have_temp_cal() else (-40, 200)
        self.t_plot = self.add_plot(
            513, limits=(-0.1, t_lim[0], 50, t_lim[1]), max_v_ticks=4,
            sharex=self.d_plot)
        self.t_lines = self.t_plot.add_lines([], width=LINE_WIDTH)

        f_min = tc.crystal_info.search_min_f
        f_max = tc.crystal_info.search_max_f
        self.fc_plot = self.add_plot(
            (5, 8, (25, 27)),
            limits=(-0.1, f_min, 50, f_max),
            max_v_ticks=4, sharex=self.d_plot)
        self.fc_lines = self.fc_plot.add_lines([], width=LINE_WIDTH)

        self.w_plot = self.add_plot(
            (5, 8, (28, 30)), limits=(-0.1, 0, 50, 100), max_v_ticks=4,
            sharex=self.d_plot)
        self.w_lines = self.w_plot.add_lines([], width=LINE_WIDTH)

        self.zx_plot = self.add_plot(
            (5, 4, 17), limits=(f_min, -1, f_max, 1),
            max_h_ticks=4, max_v_ticks=4)
        self.zx_lines = self._make_lines(self.zx_plot)

        self.phi_plot = self.add_plot(
            (5, 4, 18),
            limits=(f_min, -math.pi, f_max, math.pi),
            max_h_ticks=4, max_v_ticks=4, sharex=self.zx_plot)
        self.phi_lines = self._make_lines(self.phi_plot)

        self.rzx_plot = self.add_plot(
            (5, 4, 19), limits=(f_min, -1, f_max, 1),
            max_h_ticks=4, max_v_ticks=4, sharex=self.zx_plot)
        self.rzx_lines = self._make_lines(self.rzx_plot)
        self.rzx_lines[0].point_width = 3

        self.chirp_plot = self.add_plot(
            (5, 4, (17, 19)), limits=(15000, 0, 40000, 0.01),
            max_h_ticks=8, max_v_ticks=4, visible=False)
        self.chirp_line = self.chirp_plot.add_steps()
        self.chirp_lorentz_line = self.chirp_plot.add_lines()

        self.nyq_plot = self.add_plot(
            (5, 4, (16, 20)), max_h_ticks=4, max_v_ticks=4,
            aspect=glotlib.ASPECT_SQUARE)
        self.nyq_plot.add_hline(0, color=(0.5, 0.75, 0.5))
        self.nyq_plot.add_vline(0, color=(0.5, 0.75, 0.5))
        self.nyq_lines = self._make_lines(self.nyq_plot)
        self.nyq_lines[0].point_width = 3

        self.status_text = self.add_label((0.01, 0.01), self.drive_str)
        self.pos_label   = self.add_label((0.99, 0.01), '', anchor='SE')
        self.freq_label  = self.add_label((0.01, 0.99), '', anchor='NW')
        self.str_label   = self.add_label((0.50, 0.99), '', anchor='N')
        self.width_label = self.add_label((0.99, 0.99), '', anchor='NE')

        glotlib.periodic(1, self.update_periodic)

    @staticmethod
    def _make_lines(plot):
        lines = [plot.add_lines([], color=HISTORY_COLORS[-(i+1)],
                                width=LINE_WIDTH)
                 for i in range(HISTORY_PLOTS)]
        lines.reverse()
        return lines

    def update_periodic(self, _t):
        self.mark_dirty()

    def handle_window_size_changed(self):
        if self.view_mode == ViewMode.LARGE:
            self.large_w, self.large_h = self.w_w, self.w_h
        elif self.view_mode == ViewMode.SMALL:
            self.small_w, self.small_h = self.w_w, self.w_h

    def handle_mouse_moved(self, x, y):
        self.mark_dirty()

    def handle_key_press(self, key):
        if key == glotlib.KEY_ESCAPE:
            if self.view_mode == ViewMode.LARGE:
                self.shrink_window()
            elif self.view_mode == ViewMode.SMALL:
                self.expand_window()

    def shrink_window(self):
        if self.view_mode == ViewMode.SMALL:
            return

        self.d_plot.hide()
        self.v_plot.hide()
        self.t_plot.hide()
        self.fc_plot.hide()
        self.w_plot.hide()
        self.zx_plot.set_bounds(141, pad_b=0.2, pad_t=0.1)
        self.phi_plot.set_bounds(142, pad_b=0.2, pad_t=0.1)
        self.rzx_plot.set_bounds(143, pad_b=0.2, pad_t=0.1)
        self.chirp_plot.set_bounds((1, 4, (1, 3)), pad_b=0.2, pad_t=0.1)
        self.nyq_plot.set_bounds(144, pad_b=0.2, pad_t=0.1)
        self.view_mode = ViewMode.SMALL
        self.resize(self.small_w, self.small_h)

    def expand_window(self):
        if self.view_mode == ViewMode.LARGE:
            return

        self.d_plot.show()
        self.v_plot.show()
        self.t_plot.show()
        self.fc_plot.show()
        self.w_plot.show()
        self.zx_plot.set_bounds((5, 4, 17))
        self.phi_plot.set_bounds((5, 4, 18))
        self.rzx_plot.set_bounds((5, 4, 19))
        self.chirp_plot.set_bounds((5, 4, (17, 19)))
        self.nyq_plot.set_bounds((5, 4, (16, 20)))
        self.view_mode = ViewMode.LARGE
        self.resize(self.large_w, self.large_h)

    def set_status_text(self, t):
        return self.status_text.set_text('%s\n%s' % (self.drive_str, t))

    def update_geometry(self, _t):
        if self.track_mode == TrackMode.SWEEP:
            return self.update_sweep_geometry()
        if self.track_mode == TrackMode.CHIRP:
            return self.update_chirp_geometry()
        return False

    def update_sweep_geometry(self):
        if self.end_sweep_time is not None:
            dt = max(self.end_sweep_time - time.time(), 0)
            updated = self.set_status_text('%s: %.0f seconds' %
                                           (self.sweep_prefix, dt))
        else:
            updated = self.set_status_text('')

        _, _, _, data_x, data_y = self.get_mouse_pos()
        if data_x is not None:
            updated |= self.pos_label.set_text('%.10f  %.10f' %
                                               (data_x, data_y))

        with self.data_lock:
            if self.data_gen == self.plot_gen:
                return updated
            self.plot_gen  = self.data_gen
            sweeps         = self.sweeps[:]
            f_centers      = self.f_centers[:]
            widths         = self.widths[:]
            strengths      = self.strengths[:]
            temperatures   = self.temperatures[:]
            densities      = self.densities[:]
            viscosities    = self.viscosities[:]
            sweep_snap     = self.sweep_snap
            # tags           = self.tags[:]

            self.sweep_snap = False

        ts = list(range(len(f_centers)))
        self.fc_lines.set_x_y_data(ts, f_centers)
        self.d_lines.set_x_y_data(ts, densities)
        self.v_lines.set_x_y_data(ts, viscosities)
        self.t_lines.set_x_y_data(ts, temperatures)
        self.w_lines.set_x_y_data(ts, widths)

        for i, sweep in enumerate(sweeps):
            fs = [p.f for p in sweep]
            self.zx_lines[i].set_x_y_data(fs, [abs(p.z) for p in sweep])
            self.phi_lines[i].set_x_y_data(fs,
                                           [cmath.phase(p.z) for p in sweep])
            self.rzx_lines[i].set_x_y_data(fs, [p.z.real for p in sweep])
            self.nyq_lines[i].set_x_y_data([p.z.real for p in sweep],
                                           [-p.z.imag for p in sweep])

        freq = f_centers[-1]
        if self.chirp_freq is not None:
            self.freq_label.set_text('F: %.3f Hz' % self.chirp_freq)
        elif freq is not None:
            self.freq_label.set_text('F: %.3f Hz' % freq)
        else:
            self.freq_label.set_text('')

        strength = strengths[-1]
        if self.chirp_strength is not None:
            self.str_label.set_text('S: %.2f' % self.chirp_strength)
        elif strength is not None:
            self.str_label.set_text('S: %.3e' % strength)
        else:
            self.str_label.set_text('')

        width = widths[-1]
        if self.chirp_width is not None:
            self.width_label.set_text('W: %.3f Hz' % self.chirp_width)
        elif width is not None:
            self.width_label.set_text('W: %.3f Hz' % width)
        else:
            self.width_label.set_text('')

        if sweep_snap:
            self.zx_plot.snap_bounds()
            self.rzx_plot.snap_bounds()
            self.nyq_plot.snap_bounds()

        return True

    def update_chirp_geometry(self):
        with self.data_lock:
            if self.chirp_X_data is None:
                return False

            chirp_X_data         = self.chirp_X_data
            chirp_Y_data         = self.chirp_Y_data
            chirp_X_lorentz      = self.chirp_X_lorentz
            chirp_Y_lorentz      = self.chirp_Y_lorentz
            chirp_fit            = self.chirp_fit
            self.chirp_X_data    = None
            self.chirp_Y_data    = None
            self.chirp_X_lorentz = None
            self.chirp_Y_lorentz = None
            self.chirp_fit       = None

        self.chirp_line.set_x_y_data(chirp_X_data, chirp_Y_data)
        self.chirp_lorentz_line.set_x_y_data(chirp_X_lorentz, chirp_Y_lorentz)
        self.chirp_plot.snap_bounds()

        if self.chirp_freq is not None:
            self.freq_label.set_text('F: %.3f Hz' % self.chirp_freq)
        if self.chirp_width is not None:
            self.width_label.set_text('W: %.3f Hz' % self.chirp_width)
        if self.chirp_strength is not None:
            self.str_label.set_text('S: %.2f' % self.chirp_strength)

        if chirp_fit:
            self.set_status_text('Chirping: R**2 = %.5f' % chirp_fit.RR)
        else:
            self.set_status_text('Chirping: (No fit)')

        return True

    def data_callback(self, _sweep, fw_fit, points, T, D, V, hires):
        with self.data_lock:
            self.sweeps.pop()
            self.sweeps.insert(0, points)
            if fw_fit is not None:
                self.f_centers.append(fw_fit.peak_hz)
                self.widths.append(fw_fit.peak_fwhm)
                self.strengths.append(fw_fit.strength)
            else:
                self.f_centers.append(None)
                self.widths.append(None)
                self.strengths.append(None)
            self.temperatures.append(T)
            self.densities.append(D)
            self.viscosities.append(V)
            self.tags.append('Hires' if hires else 'Search')
            self.data_gen += 1

            self.chirp_plot.hide()
            self.zx_plot.show()
            self.phi_plot.show()
            self.rzx_plot.show()

            self.chirp_freq     = None
            self.chirp_width    = None
            self.chirp_strength = None

            self.mark_dirty()

    def chirp_callback(self, _tc, _pt, _n, lf, X_data, Y_data, X_lorentzian,
                       Y_lorentzian):
        with self.data_lock:
            self.track_mode = TrackMode.CHIRP
            self.sweep_snap = True

            self.chirp_plot.show()
            self.zx_plot.hide()
            self.phi_plot.hide()
            self.rzx_plot.hide()

            self.chirp_X_data    = X_data
            self.chirp_Y_data    = Y_data
            self.chirp_X_lorentz = X_lorentzian
            self.chirp_Y_lorentz = Y_lorentzian
            self.chirp_fit       = lf
            if lf is not None:
                self.chirp_freq     = lf.x0
                self.chirp_width    = 2 * lf.W
                self.chirp_strength = lf.A / (math.pi * lf.W)
            else:
                self.chirp_freq     = None
                self.chirp_width    = None
                self.chirp_strength = None

            self.mark_dirty()

    def sweep_started_callback(self, _tc, pt, _duration_ms, hires, f0, f1):
        with self.data_lock:
            self.track_mode     = TrackMode.SWEEP
            self.end_sweep_time = pt.t_timeout
            freq_str            = '[%.2f - %.2f]' % (f0, f1)
            if hires:
                self.sweep_prefix = 'Sweeping Hires %s' % freq_str
            else:
                self.sweep_prefix = 'Searching %s' % freq_str
            self.mark_dirty()

    def sweep_callback(self, tc, pt, t0_ns, duration_ms, points, fw_fit, hires,
                       temp_freq, temp_c):
        T, D, V = self.z_logger.log_sweep(tc, pt, t0_ns, points, fw_fit, hires,
                                          temp_freq, temp_c)
        self.data_callback(pt.sweep, fw_fit, points, T, D, V, hires)

        if self.ipq:
            p = self.make_influx_point(t0_ns, duration_ms, fw_fit, hires,
                                       temp_freq)
            self.ipq.append(p)

    def make_influx_point(self, t0_ns, duration_ms, fw_fit, hires, temp_freq):
        return self.tc.make_influx_point(t0_ns, duration_ms, fw_fit, hires,
                                         temp_freq)


def main(rv):
    _c, ipq = z_common.parse_config(rv)

    track_admittance = not rv.track_impedance

    dev    = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc     = xtalx.z_sensor.make(dev, verbose=rv.verbose,
                                 yield_Y=track_admittance)
    za, zl = z_common.parse_args(tc, rv)
    tw     = TrackerWindow(tc, ipq, za, zl, tc.serial_num)
    pt     = xtalx.z_sensor.PeakTracker(
              tc, za.amplitude, za.nfreqs,
              za.search_time_secs, za.sweep_time_secs,
              settle_ms=za.settle_ms, delegate=tw)
    pt.start_threaded()

    try:
        glotlib.interact()
    except KeyboardInterrupt:
        print()
    finally:
        pt.stop_threaded()

    if ipq:
        ipq.flush()


def _main():
    parser = argparse.ArgumentParser()
    z_common.add_arguments(parser)
    rv = parser.parse_args()
    main(rv)


if __name__ == '__main__':
    _main()
