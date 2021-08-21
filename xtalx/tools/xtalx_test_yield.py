#!/usr/bin/env python3
# Copyright (c) 2020-2021 by Phase Advanced Sensor Systems Corp.
import argparse

import xtalx


VERBOSE = False


def xtalx_cb(m):
    print(m.tostring(VERBOSE))


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

    for m in xtalx.XtalX(d).yield_measurements():
        xtalx_cb(m)


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--verbose', '-v', action='store_true')
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
