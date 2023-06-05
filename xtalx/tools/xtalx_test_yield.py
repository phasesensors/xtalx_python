#!/usr/bin/env python3
# Copyright (c) 2020-2021 by Phase Advanced Sensor Systems Corp.
import argparse
import time

import xtalx


VERBOSE = False


def xtalx_cb(m, csv_file):
    print(m.tostring(VERBOSE))
    if csv_file:
        csv_file.write('%.6f,%.2f,%.5f\n' % (
            time.time(), m.temp_c, m.pressure_psi))


def main(args):
    global VERBOSE
    VERBOSE = args.verbose

    if args.serial_number is not None:
        sensors = xtalx.find(serial_number=args.serial_number)
        if not sensors:
            print('No matching sensors.')
            for s in xtalx.find():
                print('    %s' % s.serial_number)
            return
    else:
        sensors = xtalx.find()
        if not sensors:
            print('No sensors found.')
            return
    if len(sensors) != 1:
        print('Matching sensors:')
        for s in sensors:
            print('    %s' % s.serial_number)
        return
    d = sensors[0]

    if args.csv_file:
        csv_file = open(  # pylint: disable=R1732
            args.csv_file, 'a', encoding='utf8')
        csv_file.write('time,temp_c,pressure_psi\n')
    else:
        csv_file = None

    for m in xtalx.XtalX(d).yield_measurements():
        xtalx_cb(m, csv_file)


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--csv-file')
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
