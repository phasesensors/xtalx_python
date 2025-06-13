# Copyright (c) 2023-2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import math
import time

import glotlib
import glfw

import xtalx.p_sensor
import xtalx.modbus_adapter
import xtalx.tools.modbus.serial


LINE_WIDTH = 1
RUNNING = True
TARE = 0


class TrackerWindow(glotlib.Window):
    def __init__(self, name):
        super().__init__(900, 700, msaa=4, name=name)

        self.data_gen  = -1
        self.plot_gen  = -1
        self.data_lock = threading.Lock()
        self.new_data  = []
        self.last_diff = 0

        self.p_plot = self.add_plot(
            311, limits=(-0.1, -0.5, 120, 300), max_v_ticks=10)
        self.p_lines_0 = self.p_plot.add_steps(X=[], Y=[], width=LINE_WIDTH)
        self.p_lines_1 = self.p_plot.add_steps(X=[], Y=[], width=LINE_WIDTH)
        self.p_plot.set_y_label('PSI')

        self.diff_plot = self.add_plot(
            312, limits=(-0.1, -3, 120, 3), max_v_ticks=10,
            sharex=self.p_plot)
        self.diff_lines = self.diff_plot.add_lines(X=[], Y=[], width=LINE_WIDTH)
        self.diff_plot.set_y_label('PSI')

        self.t_plot = self.add_plot(
            313, limits=(-0.1, 10, 180, 50), max_v_ticks=10,
            sharex=self.p_plot)
        self.t_lines_0 = self.t_plot.add_steps(X=[], Y=[],  width=LINE_WIDTH)
        self.t_lines_1 = self.t_plot.add_steps(X=[], Y=[],  width=LINE_WIDTH)
        self.t_plot.set_y_label('Temp (C)')

        self.pos_label = self.add_label((0.99, 0.01), '', anchor='SE')

        self.mouse_vlines = [self.p_plot.add_vline(0, color='#80C080'),
                             self.diff_plot.add_vline(0, color='#80C080'),
                             self.t_plot.add_vline(0, color='#80C080')]

        label_font = glotlib.fonts.vera_bold(48, 0)
        self.psi_label  = self.add_label(self.p_plot.bounds[2:4], '',
                                         anchor='NE', font=label_font)
        self.diff_label = self.add_label(self.diff_plot.bounds[2:4], '',
                                         anchor='NE', font=label_font)
        self.temp_label = self.add_label(self.t_plot.bounds[2:4], '',
                                         anchor='NE', font=label_font)

    def handle_mouse_moved(self, x, y):
        data_x, _ = self.p_plot._window_to_data(x, y)
        for vline in self.mouse_vlines:
            vline.set_x_data(data_x)
        self.mark_dirty()

    def handle_key_press(self, key):
        global TARE
        if key == glfw.KEY_T:
            TARE = self.last_diff
        elif key == glfw.KEY_U:
            TARE = 0

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

            X  = [m[0]._timestamp for m in new_data]
            Y0 = [m[0].pressure_psi for m in new_data]
            Y1 = [m[1].pressure_psi for m in new_data]

            # Hi-res pressure measurements.
            self.p_lines_0.append_x_y_data(X, Y0)
            self.p_lines_1.append_x_y_data(X, Y1)
            self.psi_label.set_text('%.4f PSI\n%.4f PSI' %
                                    (new_data[-1][0].pressure_psi,
                                     new_data[-1][1].pressure_psi))

            # Difference measurement.
            dY = [y0 - y1 - TARE for y0, y1 in zip(Y0, Y1)]
            self.diff_lines.append_x_y_data(X, dY)
            self.diff_label.set_text('%.4f PSI' % dY[-1])
            self.last_diff = Y0[-1] - Y1[-1]

            # Hi-res temperature measurements.
            self.t_lines_0.append_x_y_data(
                X, [m[0].temp_c for m in new_data])
            self.t_lines_1.append_x_y_data(
                X, [m[1].temp_c for m in new_data])
            self.temp_label.set_text('%.4f \u00B0C\n%.4f \u00B0C' %
                                     (new_data[-1][0].temp_c,
                                      new_data[-1][1].temp_c))

        return updated

    def measurement_callback(self, m0, m1):
        with self.data_lock:
            self.new_data.append((m0, m1))
            self.mark_dirty()


def sleep_until(t0, dt=0):
    '''
    Sleeps until time (t0 + dt).
    '''
    while True:
        rem = dt - (time.time() - t0)
        if rem <= 0:
            return
        time.sleep(rem)


def measure_thread(xs, tw, csv_file, poll_rate_secs):
    t0 = time.time()
    while RUNNING:
        t = time.time()
        m0 = xs[0].read_measurement()
        m1 = xs[1].read_measurement()
        m0._timestamp = m1._timestamp = dt = t - t0
        tw.measurement_callback(m0, m1)

        if csv_file:
            temp_c0 = m0.temp_c if m0.temp_c is not None else math.nan
            pressure_psi0 = (m0.pressure_psi if m0.pressure_psi is not None
                             else math.nan)
            temp_c1 = m1.temp_c if m1.temp_c is not None else math.nan
            pressure_psi1 = (m1.pressure_psi if m1.pressure_psi is not None
                             else math.nan)
            diff_psi = pressure_psi0 - pressure_psi1
            csv_file.write('%.6f,%.6f,%.5f,%.5f,%.5f,%.2f,%.5f,%.2f,%.5f\n' % (
                t, dt, diff_psi, TARE, diff_psi - TARE, temp_c0, pressure_psi0,
                temp_c1, pressure_psi1))
            csv_file.flush()

        sleep_until(t, poll_rate_secs)


def make_sensors(args):
    dev = xtalx.modbus_adapter.find_one_mba()
    if dev is None:
        raise Exception('No matching devices.')

    bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=args.baud_rate)
    bus.set_vext(True)
    time.sleep(0.1)
    return [xtalx.p_sensor.XHTISM(bus, int(addr, 0))
            for addr in args.modbus_addr]


def main(args):
    global RUNNING

    if len(args.modbus_addr) != 2:
        print('Must specify exactly two sensors.')
        return

    xs = make_sensors(args)

    if args.csv_file:
        hdr = 'Time (s)'
        hdr += ',Delta Time (s)'
        hdr += ',Pressure Difference (PSI)'
        hdr += ',Tare Value (PSI)'
        hdr += ',Tared Pressure Difference (PSI)'
        hdr += ',' + xs[0].serial_num + ' Temp (C)'
        hdr += ',' + xs[0].serial_num + ' Pressure (PSI)'
        hdr += ',' + xs[1].serial_num + ' Temp (C)'
        hdr += ',' + xs[1].serial_num + ' Pressure (PSI)'
        hdr += '\n'

        csv_file = open(  # pylint: disable=R1732
            args.csv_file, 'a+', encoding='utf8')

        pos = csv_file.tell()
        if pos != 0:
            csv_file.seek(0)
            if csv_file.read(len(hdr)) != hdr:
                print('%s does not appear to be a difference log file.' %
                      args.csv_file)
                return
            csv_file.seek(pos)
        else:
            csv_file.write(hdr)
            csv_file.flush()
    else:
        csv_file = None

    tw  = TrackerWindow(xs[0].serial_num + ' - ' + xs[1].serial_num)
    mt  = threading.Thread(target=measure_thread,
                           args=(xs, tw, csv_file, float(args.poll_rate_secs)))
    mt.start()

    try:
        glotlib.interact()
    except KeyboardInterrupt:
        print()
    finally:
        RUNNING = False
        mt.join()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baud-rate', type=int, default=115200)
    parser.add_argument('--modbus-addr', '-m', action='append')
    parser.add_argument('--csv-file')
    parser.add_argument('--poll-rate-secs', default=0)
    args = parser.parse_args()
    main(args)


if __name__ == '__main__':
    _main()
