#!/usr/bin/env python3
# Copyright (c) 2020-2024 by Phase Advanced Sensor Systems Corp.
import argparse

import xtalx.p_sensor


def main(args):
    d = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)
    x = xtalx.p_sensor.make_xti(d)
    while True:
        m = x.read_measurement()
        print(m.tostring(args.verbose))


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
