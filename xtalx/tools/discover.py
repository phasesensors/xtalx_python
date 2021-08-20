#!/usr/bin/env python3
# Copyright (c) 2020-2021 by Phase Advanced Sensor Systems Corp.
import argparse
import usb.util

import xtalx


def main(_args):
    for s in xtalx.find():
        print('******************')
        print('Sensor SN: %s' % s.serial_number)
        print(' git SHA1: %s' % usb.util.get_string(s, 6))
        print('  Version: 0x%04X' % s.bcdDevice)


def _main():
    parser = argparse.ArgumentParser()
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
