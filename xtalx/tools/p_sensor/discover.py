#!/usr/bin/env python3
# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import argparse
import usb.util

import xtalx.p_sensor


def main(_args):
    for s in xtalx.p_sensor.find_xti():
        print('******************')
        print('Sensor SN: %s' % s.serial_number)
        print(' git SHA1: %s' % usb.util.get_string(s, 6))
        print('  Version: 0x%04X' % s.bcdDevice)


def _main():
    parser = argparse.ArgumentParser()
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
