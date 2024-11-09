# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import cmath
import time
from enum import Enum

import glotlib
import glfw

import xtalx.z_sensor
from xtalx.z_sensor.peak_tracker import Delegate


# Search parameters.
PT_NFREQS          = 20
PT_SEARCH_TIME_SEC = 5
ENABLE_CHIRP       = False

# Sweep parameters.
PT_SWEEP_TIME_SEC  = 5
PT_SETTLE_TIME_MS  = 5000


class ViewMode(Enum):
    F_VS_T      = 1
    F_T_VS_TIME = 2


class WATWindow(glotlib.Window, Delegate):
    def __init__(self, tc, csv):
        super().__init__(1000, 700, msaa=4, name=tc.serial_num)

        self.tc             = tc
        self.csv            = csv
        self.data_lock      = threading.Lock()
        self.fits           = []
        self.points         = None
        self.auto_snap      = True
        self.end_sweep_time = None
        self.sweep_prefix   = ''
        self.view_mode      = ViewMode.F_VS_T

        self.f_vs_T_plot = self.add_plot((3, 4, (1, 11)),
                                         limits=(0, 27000, 130, 29500))
        self.f_vs_T_lines = self.f_vs_T_plot.add_lines(X=[], Y=[])
        self.f_vs_T_point = self.f_vs_T_plot.add_points(X=[], Y=[], width=3)
        self.f_vs_T_plot.set_x_label('Temperature (C)')
        self.f_vs_T_plot.set_y_label('Freq (Hz)')

        self.t_vs_time_plot = self.add_plot((2, 4, (1, 3)),
                                            limits=(-1, 0, 60, 130),
                                            visible=False)
        self.t_vs_time_lines = self.t_vs_time_plot.add_lines(X=[], Y=[])
        self.t_vs_time_plot.set_y_label('Temperature (C)')

        self.f_vs_time_plot = self.add_plot((2, 4, (5, 7)),
                                            limits=(-1, 27000, 60, 29500),
                                            visible=False,
                                            sharex=self.t_vs_time_plot)
        self.f_vs_time_lines = self.f_vs_time_plot.add_lines(X=[], Y=[])
        self.f_vs_time_plot.set_x_label('Time (min)')
        self.f_vs_time_plot.set_y_label('Freq (Hz)')

        self.rzx_plot = self.add_plot((3, 4, (4, 4)), max_h_ticks=3)
        self.rzx_lines = self.rzx_plot.add_lines(X=[], Y=[])
        self.rzx_plot.set_x_label('Freq (Hz)', side='top')
        self.rzx_plot.set_y_label('Real(Z)', side='right')

        self.phi_plot = self.add_plot((3, 4, (8, 8)), max_h_ticks=3,
                                      sharex=self.rzx_plot)
        self.phi_lines = self.phi_plot.add_lines(X=[], Y=[])
        self.phi_plot.set_y_label('Arg(Z)', side='right')

        self.nyq_plot = self.add_plot((3, 4, (12, 12)), max_h_ticks=3,
                                      aspect=glotlib.ASPECT_SQUARE)
        self.nyq_lines = self.nyq_plot.add_lines(X=[], Y=[])
        self.nyq_plot.set_x_label('Real(Z)')
        self.nyq_plot.set_y_label('-Imag(Z)', side='right')

        self.status_text = self.add_label((0.01, 0.01), '')
        self.pos_label = self.add_label((0.99, 0.01), '', anchor='SE')
        self.temp_label = self.add_label(
                (self.f_vs_T_plot.bounds[0] + 0.06,
                 self.f_vs_T_plot.bounds[3] + 0.01),
                '', anchor='SW')
        self.freq_label = self.add_label(
                (self.f_vs_T_plot.bounds[2] - 0.01,
                 self.f_vs_T_plot.bounds[3] + 0.01),
                '', anchor='SE')

        glotlib.periodic(1, self.update_periodic)

    def update_periodic(self, _t):
        self.mark_dirty()

    def handle_mouse_moved(self, _x, _y):
        self.mark_dirty()

    def handle_key_press(self, key):
        if key == glotlib.KEY_ESCAPE:
            self.swap_view()
        elif key == glfw.KEY_Z:
            self.default_zoom()

    def swap_view(self):
        if self.view_mode == ViewMode.F_VS_T:
            self.f_vs_T_plot.hide()
            self.f_vs_time_plot.show()
            self.t_vs_time_plot.show()
            self.view_mode = ViewMode.F_T_VS_TIME
        elif self.view_mode == ViewMode.F_T_VS_TIME:
            self.f_vs_T_plot.show()
            self.f_vs_time_plot.hide()
            self.t_vs_time_plot.hide()
            self.view_mode = ViewMode.F_VS_T

    def default_zoom(self):
        self.f_vs_T_plot._set_x_lim(0, 130)
        self.f_vs_T_plot._set_y_lim(27000, 29500)
        self.t_vs_time_plot._set_y_lim(0, 130)
        self.f_vs_time_plot._set_y_lim(27000, 29500)

    def set_status_text(self, t):
        return self.status_text.set_text('%s' % t)

    def update_geometry(self, _t):
        with self.data_lock:
            fits        = self.fits
            points      = self.points

            if fits:
                self.fits = []
            self.points = None

        updated = False

        _, _, _, data_x, data_y = self.get_mouse_pos()
        if data_x is not None:
            updated |= self.pos_label.set_text('%.4f  %.4f' %
                                               (data_x, data_y))

        if self.end_sweep_time is not None:
            dt = max(self.end_sweep_time - time.time(), 0)
            if dt == 0:
                updated |= self.set_status_text('%s: Fitting...' %
                                                self.sweep_prefix)
            else:
                updated |= self.set_status_text('%s: %.0f seconds' %
                                                (self.sweep_prefix, dt))
        else:
            updated |= self.set_status_text('')

        if fits:
            T = [fit._gl_dtime_ns / 60e9 for fit in fits]
            X = [fit.temp_c for fit in fits]
            Y = [fit.peak_hz for fit in fits]
            self.f_vs_T_lines.append_x_y_data(X, Y)
            self.f_vs_T_point.set_x_y_data(X=[X[-1]], Y=[Y[-1]])
            self.t_vs_time_lines.append_x_y_data(T, X)
            self.f_vs_time_lines.append_x_y_data(T, Y)
            if X[-1] is not None:
                self.temp_label.set_text('Temp: %.3fC' % X[-1])
            else:
                self.temp_label.set_text('Temp: n/a')
            self.freq_label.set_text('Freq: %.3f Hz' % Y[-1])
            updated = True

        if points:
            X = [p.f for p in points]
            Y = [cmath.phase(p.z) for p in points]
            self.phi_lines.set_x_y_data(X, Y)

            Y = [p.z.real for p in points]
            self.rzx_lines.set_x_y_data(X, Y)

            X = Y
            Y = [-p.z.imag for p in points]
            self.nyq_lines.set_x_y_data(X, Y)

            updated = True

            if self.auto_snap:
                self.rzx_plot.snap_bounds()
                self.phi_plot.snap_bounds()
                self.nyq_plot.snap_bounds()
                self.auto_snap = False

        return updated

    def sweep_started_callback(self, _tc, pt, _duration_ms, hires, f0, f1):
        with self.data_lock:
            self.end_sweep_time = pt.t_timeout
            freq_str            = '[%.2f - %.2f]' % (f0, f1)
            if hires:
                self.sweep_prefix = 'Sweeping Hires %s' % freq_str
            else:
                self.sweep_prefix = 'Searching %s' % freq_str
            self.mark_dirty()

    def sweep_callback(self, _tc, pt, t0_ns, duration_ms, points, fw_fit,
                       hires, _temp_freq, _temp_c):
        with self.data_lock:
            if pt.sweep_iter > 1:
                fw_fit._gl_time_ns  = t0_ns + duration_ms * 1000
                fw_fit._gl_dtime_ns = fw_fit._gl_time_ns - pt.start_time_ns
                self.fits.append(fw_fit)
            self.points = points
            self.mark_dirty()

        if pt.sweep_iter > 1 and self.csv:
            self.csv.write('%u,%u,%.3f,%.3f\n' %
                           (fw_fit._gl_time_ns, fw_fit._gl_dtime_ns,
                            fw_fit.temp_c, fw_fit.peak_hz))
            self.csv.flush()


def main(args):
    if args.output_csv:
        csv = open(  # pylint: disable=R1732
                args.output_csv, 'a', encoding='utf8')
        csv.write('time_ns,dt,temp_c,peak_hz\n')
        csv.flush()
    else:
        csv = None

    dev    = xtalx.z_sensor.find_one(serial_number=args.sensor)
    tc     = xtalx.z_sensor.make(dev, yield_Y=True)
    ww     = WATWindow(tc, csv)
    a      = tc.parse_amplitude(None)
    pt     = xtalx.z_sensor.PeakTracker(tc, a, PT_NFREQS, PT_SEARCH_TIME_SEC,
                                        PT_SWEEP_TIME_SEC,
                                        settle_ms=PT_SETTLE_TIME_MS,
                                        delegate=ww, enable_chirp=ENABLE_CHIRP)
    pt.start_threaded()

    try:
        glotlib.interact()
    except KeyboardInterrupt:
        print()
    finally:
        pt.stop_threaded()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-csv', '-o')
    parser.add_argument('--sensor', '-s')
    args = parser.parse_args()
    main(args)


if __name__ == '__main__':
    _main()
