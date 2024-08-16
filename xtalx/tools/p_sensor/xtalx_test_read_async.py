#!/usr/bin/env python3
# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import argparse

import xtalx.p_sensor


VERBOSE = False


def xtalx_cb(m):
    print(m.tostring(VERBOSE))


def main(args):
    global VERBOSE
    VERBOSE = args.verbose

    d = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)
    x = xtalx.p_sensor.make_xti(d)
    x.read_measurements(xtalx_cb)
    try:
        x.join_read()
    except KeyboardInterrupt:
        x.halt_read()
        raise


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
