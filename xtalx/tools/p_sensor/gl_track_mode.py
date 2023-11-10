# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import math
import time

import glotlib

import xtalx.p_sensor
from xtalx.tools.math import XYSeries


LINE_WIDTH = 1


class TrackerWindow(glotlib.Window):
    def __init__(self, name, period):
        super().__init__(900, 700, msaa=4, name=name)

        self.period         = period
        self.data_gen       = -1
        self.plot_gen       = -1
        self.data_lock      = threading.Lock()
        self.new_data       = []
        self.p_measurements = XYSeries([], [])
        self.t_measurements = XYSeries([], [])

        self.p_plot = self.add_plot(
            311, limits=(-0.1, -0.5, 120, 300), max_v_ticks=10)
        self.p_lines = self.p_plot.add_lines([], width=LINE_WIDTH)

        self.p_slow_plot = self.add_plot(
            312, limits=(-0.1, -0.5, 120, 300), max_v_ticks=10,
            sharex=self.p_plot, sharey=self.p_plot)
        self.p_slow_lines = self.p_slow_plot.add_lines([], width=LINE_WIDTH)

        self.t_plot = self.add_plot(
            313, limits=(-0.1, -0.5, 120, 50), max_v_ticks=10,
            sharex=self.p_plot)
        self.t_lines = self.t_plot.add_lines([], width=LINE_WIDTH)

        self.pos_label = self.add_label((0.99, 0.01), '', anchor='SE')

        y0 = (self.p_plot.bounds[1] + self.p_plot.bounds[3]) / 2
        y1 = (self.p_slow_plot.bounds[1] + self.p_slow_plot.bounds[3]) / 2
        y2 = (self.t_plot.bounds[1] + self.t_plot.bounds[3]) / 2
        self.add_label((.035, y0), 'PSI', anchor='NW', theta=math.pi/2)
        self.add_label((.035, y1), 'PSI (%u-sec Avg)' % period, anchor='NW',
                       theta=math.pi/2)
        self.add_label((.035, y2), 'Temp (C)', anchor='NW', theta=math.pi/2)

        self.mouse_vlines = [self.p_plot.add_vline(0, color='#80C080'),
                             self.p_slow_plot.add_vline(0, color='#80C080'),
                             self.t_plot.add_vline(0, color='#80C080')]

    def handle_mouse_moved(self, x, y):
        data_x, _ = self.p_plot._window_to_data(x, y)
        for vline in self.mouse_vlines:
            vline.set_x_data(data_x)
        self.mark_dirty()

    def update_geometry(self, _t):
        updated = False

        _, _, _, data_x, data_y = self.get_mouse_pos()
        if data_x is not None:
            updated |= self.pos_label.set_text('%.10f  %.10f' %
                                               (data_x, data_y))

        new_data = None
        with self.data_lock:
            if self.new_data:
                new_data = self.new_data
                self.new_data = []

        if new_data:
            updated = True

            self.t_measurements.append(
                [m._timestamp for m in new_data],
                [m.temp_c for m in new_data])
            self.t_lines.set_x_y_data(self.t_measurements.X,
                                      self.t_measurements.Y)

            self.p_measurements.append(
                [m._timestamp for m in new_data],
                [m.pressure_psi for m in new_data])
            self.p_lines.set_x_y_data(self.p_measurements.X,
                                      self.p_measurements.Y)

            timestamps = []
            pressures  = []
            for i in range(0, math.ceil(self.p_measurements.X[-1]),
                           self.period):
                timestamps.append(i + self.period)
                pressures.append(
                    self.p_measurements.get_avg_value(i, i + self.period))
            self.p_slow_lines.set_x_y_data(timestamps, pressures)

        return updated

    def measurement_callback(self, m):
        with self.data_lock:
            self.new_data.append(m)
            self.mark_dirty()


def measure_thread(x, tw, csv_file):
    t0 = time.time()
    for m in x.yield_measurements():
        t = time.time()
        m._timestamp = dt = t - t0
        tw.measurement_callback(m)

        if csv_file:
            temp_c = m.temp_c if m.temp_c is not None else math.nan
            pressure_psi = (m.pressure_psi if m.pressure_psi is not None
                            else math.nan)
            csv_file.write('%.6f,%.6f,%.2f,%.5f\n' % (t, dt, temp_c,
                                                      pressure_psi))
            csv_file.flush()


def main(args):
    dev = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)

    if args.csv_file:
        csv_file = open(  # pylint: disable=R1732
            args.csv_file, 'a+', encoding='utf8')

        pos = csv_file.tell()
        if pos != 0:
            csv_file.seek(0)
            if csv_file.read(28) != 'time,dt,temp_c,pressure_psi\n':
                print('%s does not appear to be a pressure sensor log file.' %
                      args.csv_file)
                return
            csv_file.seek(pos)
        else:
            csv_file.write('time,dt,temp_c,pressure_psi\n')
            csv_file.flush()
    else:
        csv_file = None

    x   = xtalx.p_sensor.make(dev)
    tw  = TrackerWindow(x.serial_num, args.averaging_period_secs)
    mt  = threading.Thread(target=measure_thread, args=(x, tw, csv_file))
    mt.start()

    try:
        glotlib.interact()
    except KeyboardInterrupt:
        print()
    finally:
        x.halt_yield()
        mt.join()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial_number', '-s')
    parser.add_argument('--csv-file')
    parser.add_argument('--averaging-period-secs', type=int, default=3)
    args = parser.parse_args()
    main(args)


if __name__ == '__main__':
    _main()
