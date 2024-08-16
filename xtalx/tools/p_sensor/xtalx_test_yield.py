#!/usr/bin/env python3
# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import argparse
import time
import math

import xtalx.p_sensor


def print_measurement(x, temp_c, pressure_psi):
    if pressure_psi is None:
        p = 'n/a'
    else:
        p = '%f' % pressure_psi

    if temp_c is None:
        t = 'n/a'
    else:
        t = '%f' % temp_c

    print('%s: %s C, %s PSI' % (x, t, p))


def xtalx_cb(x, t, dt, temp_c, pressure_psi, csv_file):
    print_measurement(x, temp_c, pressure_psi)
    if csv_file:
        if temp_c is None:
            temp_c = math.nan
        if pressure_psi is None:
            pressure_psi = math.nan
        csv_file.write('%.6f,%.6f,%.2f,%.5f\n' % (t, dt, temp_c, pressure_psi))
        csv_file.flush()


def sample_gated(x, csv_file, sample_period):
    measurements   = []
    t0 = t0_period = time.time()
    for m in x.yield_measurements():
        t = time.time()
        if t - t0_period < sample_period:
            measurements.append(m)
            continue

        if measurements:
            N      = len(measurements)
            temp_c = sum(m.temp_c for m in measurements) / N
            psi    = sum(m.pressure_psi for m in measurements) / N
            xtalx_cb(x, t, t - t0, temp_c, psi, csv_file)

        t0_period   += sample_period
        measurements = [m]


def sample_continuous(x, csv_file):
    t0 = time.time()
    for m in x.yield_measurements():
        t = time.time()
        xtalx_cb(x, t, t - t0, m.temp_c, m.pressure_psi, csv_file)


def main(args):
    if args.xhti_intf:
        x = xtalx.p_sensor.XHTI(args.xhti_intf)
    else:
        d = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)
        x = xtalx.p_sensor.make_xti(d)

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

    if args.sample_period:
        sample_gated(x, csv_file, args.sample_period)
    else:
        sample_continuous(x, csv_file)


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--xhti-intf')
    parser.add_argument('--csv-file')
    parser.add_argument('--sample-period', type=float)
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
