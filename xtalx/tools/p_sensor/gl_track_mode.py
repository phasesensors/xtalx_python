# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import math
import time

import glotlib

import xtalx.p_sensor
import xtalx.modbus_adapter
import xtalx.spi_adapter
import xtalx.tools.modbus.serial
from xtalx.tools.math import XYSeries


LINE_WIDTH = 1


class TrackerWindow(glotlib.Window):
    def __init__(self, name, period, show_lores_data, display_frequencies):
        super().__init__(900, 700, msaa=4, name=name)

        self.period              = period
        self.show_lores_data     = show_lores_data
        self.display_frequencies = display_frequencies
        self.data_gen            = -1
        self.plot_gen            = -1
        self.data_lock           = threading.Lock()
        self.new_data            = []
        self.p_measurements      = XYSeries([], [])
        self.lp_measurements     = XYSeries([], [])

        if display_frequencies:
            self.p_plot = self.add_plot(
                311, limits=(-0.1, 47000, 120, 62000), max_v_ticks=10)
        else:
            self.p_plot = self.add_plot(
                311, limits=(-0.1, -0.5, 120, 300), max_v_ticks=10)
        self.lp_lines = self.p_plot.add_steps(X=[], Y=[], width=LINE_WIDTH)
        self.p_lines = self.p_plot.add_steps(X=[], Y=[], width=LINE_WIDTH)
        self.lp_lines.color = (0.78, 0.87, 0.93, 1)
        self.p_plot.set_y_label('P Hz' if display_frequencies else 'PSI')

        if display_frequencies:
            self.p_slow_plot = self.add_plot(
                312, limits=(-0.1, 47000, 120, 62000), max_v_ticks=10,
                sharex=self.p_plot, sharey=self.p_plot)
        else:
            self.p_slow_plot = self.add_plot(
                312, limits=(-0.1, -0.5, 120, 300), max_v_ticks=10,
                sharex=self.p_plot, sharey=self.p_plot)
        self.lp_slow_lines = self.p_slow_plot.add_lines(X=[], Y=[],
                                                        width=LINE_WIDTH)
        self.p_slow_lines = self.p_slow_plot.add_lines(X=[], Y=[],
                                                       width=LINE_WIDTH)
        self.p_slow_plot.set_y_label(
            ('P Hz (%u-sec Avg)' % period) if display_frequencies else
            ('PSI (%u-sec Avg)' % period))

        if display_frequencies:
            self.t_plot = self.add_plot(
                313, limits=(-0.1, 260000, 120, 264000), max_v_ticks=10,
                sharex=self.p_plot)
        else:
            self.t_plot = self.add_plot(
                313, limits=(-0.1, -0.5, 120, 50), max_v_ticks=10,
                sharex=self.p_plot)
        self.lt_lines = self.t_plot.add_steps(X=[], Y=[], width=LINE_WIDTH)
        self.t_lines = self.t_plot.add_steps(X=[], Y=[],  width=LINE_WIDTH)
        self.lt_lines.color = (0.78, 0.87, 0.93, 1)
        self.t_plot.set_y_label('Temp Hz' if display_frequencies else
                                'Temp (C)')

        self.pos_label = self.add_label((0.99, 0.01), '', anchor='SE')

        self.mouse_vlines = [self.p_plot.add_vline(0, color='#80C080'),
                             self.p_slow_plot.add_vline(0, color='#80C080'),
                             self.t_plot.add_vline(0, color='#80C080')]

        label_font = glotlib.fonts.vera_bold(48, 0)
        self.psi_label      = self.add_label(self.p_plot.bounds[2:4], '',
                                             anchor='NE', font=label_font)
        self.psi_slow_label = self.add_label(self.p_slow_plot.bounds[2:4], '',
                                             anchor='NE', font=label_font)
        self.temp_label     = self.add_label(self.t_plot.bounds[2:4], '',
                                             anchor='NE', font=label_font)

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

            # Low-res temperature measurements.
            if self.display_frequencies:
                X = [m._timestamp for m in new_data
                     if m.lores_temp_freq is not None]
                Y = [m.lores_temp_freq for m in new_data
                     if m.lores_temp_freq is not None]
            else:
                X = [m._timestamp for m in new_data
                     if m.lores_temp_c is not None]
                Y = [m.lores_temp_c for m in new_data
                     if m.lores_temp_c is not None]
            if self.show_lores_data:
                self.lt_lines.append_x_y_data(X, Y)

            # Hi-res temperature measurements.
            if self.display_frequencies:
                self.t_lines.append_x_y_data(
                    [m._timestamp for m in new_data],
                    [m.temp_freq for m in new_data])
                self.temp_label.set_text('%.4f Hz' % new_data[-1].temp_freq)
            else:
                self.t_lines.append_x_y_data(
                    [m._timestamp for m in new_data],
                    [m.temp_c for m in new_data])
                self.temp_label.set_text('%.4f \u00B0C' % new_data[-1].temp_c)

            # Low-res pressure (LP) measurements.
            if self.display_frequencies:
                X = [m._timestamp for m in new_data
                     if m.lores_pressure_freq is not None]
                Y = [m.lores_pressure_freq for m in new_data
                     if m.lores_pressure_freq is not None]
            else:
                X = [m._timestamp for m in new_data
                     if m.lores_pressure_psi is not None]
                Y = [m.lores_pressure_psi for m in new_data
                     if m.lores_pressure_psi is not None]
            lp_len = len(self.lp_slow_lines.vertices)
            if lp_len:
                lp_timestamp = self.lp_measurements.X[-1]
            self.lp_measurements.append(X, Y)
            if self.show_lores_data:
                self.lp_lines.append_x_y_data(X, Y)

            # Averaged data from LP measurements.
            if len(X):
                if lp_len:
                    t0    = int(lp_timestamp // self.period) * self.period
                    index = lp_len - 1
                else:
                    t0    = int(X[0] // self.period) * self.period
                    index = 0

                timestamps = []
                pressures  = []
                t          = t0
                while t <= X[-1]:
                    p = self.lp_measurements.get_avg_value(t, t + self.period)
                    if p is not None:
                        timestamps.append(t + self.period / 2)
                        pressures.append(p)
                    t += self.period
                if self.show_lores_data:
                    self.lp_slow_lines.sub_x_y_data(index, timestamps,
                                                    pressures)

            # Hi-res pressure (P) measurements.
            if self.display_frequencies:
                X = [m._timestamp for m in new_data]
                Y = [m.pressure_freq for m in new_data]
                self.psi_label.set_text('%.4f Hz' % new_data[-1].pressure_freq)
            else:
                X = [m._timestamp for m in new_data]
                Y = [m.pressure_psi for m in new_data]
                self.psi_label.set_text('%.4f PSI' % new_data[-1].pressure_psi)
            p_len = len(self.p_slow_lines.vertices)
            if p_len:
                p_timestamp = self.p_measurements.X[-1]
            self.p_measurements.append(X, Y)
            self.p_lines.append_x_y_data(X, Y)

            # Averaged data from P measurements.
            if p_len:
                t0    = int(p_timestamp // self.period) * self.period
                index = p_len - 1
            else:
                t0    = int(X[0] // self.period) * self.period
                index = 0

            timestamps = []
            pressures  = []
            t          = t0
            while t <= X[-1]:
                p = self.p_measurements.get_avg_value(t, t + self.period)
                if p is not None:
                    timestamps.append(t + self.period / 2)
                    pressures.append(p)
                t += self.period
            self.p_slow_lines.sub_x_y_data(index, timestamps, pressures)
            if len(pressures) >= 2:
                if self.display_frequencies:
                    self.psi_slow_label.set_text('%.4f Hz' % pressures[-2])
                else:
                    self.psi_slow_label.set_text('%.4f PSI' % pressures[-2])

        return updated

    def measurement_callback(self, m):
        with self.data_lock:
            self.new_data.append(m)
            self.mark_dirty()


def measure_thread(x, tw, csv_file, display_frequencies):
    t0 = time.time()
    for m in x.yield_measurements(do_reset=False):
        t = time.time()
        m._timestamp = dt = t - t0
        tw.measurement_callback(m)

        if csv_file:
            if display_frequencies:
                temp = m.temp_freq if m.temp_freq is not None else math.nan
                pressure = (m.pressure_freq if m.pressure_freq is not None
                            else math.nan)
            else:
                temp = m.temp_c if m.temp_c is not None else math.nan
                pressure = (m.pressure_psi if m.pressure_psi is not None
                            else math.nan)
            csv_file.write('%.6f,%.6f,%.5f,%.5f\n' % (t, dt, temp, pressure))
            csv_file.flush()


def make_sensor(args):
    if args.intf:
        bus = xtalx.tools.modbus.serial.Bus(args.intf, args.baud_rate)
        return xtalx.p_sensor.XHTISM(bus, int(args.modbus_addr, 0))

    dev = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)
    if dev is not None:
        return xtalx.p_sensor.make(dev)

    dev = xtalx.modbus_adapter.find_one_mba(serial_number=args.serial_number)
    if dev is not None:
        bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=args.baud_rate)
        bus.set_vext(True)
        time.sleep(0.1)
        return xtalx.p_sensor.XHTISM(bus, int(args.modbus_addr, 0))

    dev = xtalx.spi_adapter.find_one_spia(serial_number=args.serial_number)
    if dev is not None:
        bus = xtalx.spi_adapter.make_spia(dev)
        bus.set_vext(True)
        time.sleep(0.1)
        return xtalx.p_sensor.XHTISS(bus)

    raise Exception('No matching devices.')


def main(args):
    x = make_sensor(args)

    if args.csv_file:
        if args.display_frequencies:
            hdr = 'time,dt,temp_hz,pressure_hz\n'
        else:
            hdr = 'time,dt,temp_c,pressure_psi\n'
        csv_file = open(  # pylint: disable=R1732
            args.csv_file, 'a+', encoding='utf8')

        pos = csv_file.tell()
        if pos != 0:
            csv_file.seek(0)
            if csv_file.read(28) != hdr:
                print('%s does not appear to be a pressure sensor log file.' %
                      args.csv_file)
                return
            csv_file.seek(pos)
        else:
            csv_file.write(hdr)
            csv_file.flush()
    else:
        csv_file = None

    tw  = TrackerWindow(x.serial_num, args.averaging_period_secs,
                        args.show_lores_data, args.display_frequencies)
    mt  = threading.Thread(target=measure_thread,
                           args=(x, tw, csv_file, args.display_frequencies))
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
    parser.add_argument('--intf', '-i')
    parser.add_argument('--baud-rate', type=int, default=115200)
    parser.add_argument('--modbus-addr', '-m', default='0x80')
    parser.add_argument('--serial_number', '-s')
    parser.add_argument('--csv-file')
    parser.add_argument('--averaging-period-secs', type=int, default=3)
    parser.add_argument('--show-lores-data', action='store_true')
    parser.add_argument('--display-frequencies', action='store_true')
    args = parser.parse_args()
    main(args)


if __name__ == '__main__':
    _main()
