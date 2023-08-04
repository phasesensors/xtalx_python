#!/usr/bin/env python3
# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import usb.util

import xtalx.z_sensor


def main():
    for s in xtalx.z_sensor.find():
        print('******************')
        print('  Product: %s' % s.product)
        print('Sensor SN: %s' % s.serial_number)
        print(' git SHA1: %s' % usb.util.get_string(s, 4))
        print('  Version: 0x%04X' % s.bcdDevice)


if __name__ == '__main__':
    main()
