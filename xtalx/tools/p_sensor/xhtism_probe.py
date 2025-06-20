#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import time
import sys

import xtalx.modbus_adapter
from xtalx.tools.iter import prange


# Baud rates to probe.
DEFAULT_BAUD_RATES = [115200, 57600]
EXCLUDE_BAUD_RATES = []
BAUD_RATES = DEFAULT_BAUD_RATES + [
    4800*i for i in range(1, 256)
    if 4800*i not in (DEFAULT_BAUD_RATES + EXCLUDE_BAUD_RATES)
]


def main(args):
    # Find the XtalX Modbus adapter.
    dev = xtalx.modbus_adapter.find_one_mba(
            serial_number=args.modbus_adapter_serial_number)
    if dev is None:
        print('No XtalX Modbus adapter found.')
        return

    # Make the bus.
    bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=BAUD_RATES[0],
                                        parity=args.parity)

    # Search.
    print('Searching...')
    found_devices = {}
    try:
        rem_sensors = args.num_sensors
        while rem_sensors > 0:
            while BAUD_RATES:
                if rem_sensors == 0:
                    break
                baud_rate = BAUD_RATES.pop(0)
                print('Trying %u baud.' % baud_rate)
                bus.set_comm_params(baud_rate, args.parity)
                bus.set_vext(False)
                time.sleep(1)
                bus.set_vext(True)
                time.sleep(1)

                for addr in prange(1, 248, clear_line=True):
                    try:
                        objs = bus.read_device_identification(
                            addr, 0x01, 0x00,
                            response_time_ms=args.response_timeout_ms)
                    except xtalx.tools.modbus.ResponseTimeoutException:
                        continue
                    except xtalx.tools.modbus.ModbusException as e:
                        sys.stdout.write('\r**** Device 0x%02X error: '
                                         '%s\x1B[0K\r\n'
                                         % (addr, str(type(e)) + str(e)))
                        continue
                    except xtalx.modbus_adapter.CommandException as e:
                        sys.stdout.write('\r**** Device 0x%02X error: '
                                         '%s\x1B[0K\r\n'
                                         % (addr, str(type(e)) + str(e)))
                        continue

                    sys.stdout.write('\r**** Device 0x%02X found: %s '
                                     '%s\x1B[0K\r\n'
                                     % (addr, objs[0].value, objs[1].value))
                    found_devices[addr] = (baud_rate, objs)

                    rem_sensors -= 1
                    if rem_sensors == 0:
                        break
    except KeyboardInterrupt:
        print()

    print('Device summary')
    print('==============')
    if found_devices:
        for k, v in found_devices.items():
            baud_rate, objs = v
            print('0x%02X @ %u baud: %s %s'
                  % (k, baud_rate, objs[0].value, objs[1].value))
    else:
        print('No devices found.')


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--modbus-adapter-serial-number', '-s')
    parser.add_argument('--parity', default='E', choices=['E', 'O', 'N'])
    parser.add_argument('--num-sensors', '-n', type=int, default=1)
    parser.add_argument('--response-timeout-ms', '-t', type=int, default=100)

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
