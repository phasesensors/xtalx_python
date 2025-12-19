#!/usr/bin/env python3
# Copyright (c) 2025 by Phase Advanced Sensor Systems Corp.
import argparse
import threading
import time

import xtalx.p_sensor
import xtalx.modbus_adapter
import xtalx.spi_adapter
import xtalx.tools.modbus.serial


def measure_thread(x):
    for m in x.yield_measurements(do_reset=False):
        print(m.tostring(verbose=True))


def make_sensor(args):
    if args.intf:
        bus = xtalx.tools.modbus.serial.Bus(args.intf, args.baud_rate)
        x = xtalx.p_sensor.XHTISM(bus, int(args.modbus_addr, 0))
        if args.serial_number is None or x.serial_num == args.serial_number:
            return x

    dev = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)
    if dev is not None:
        return xtalx.p_sensor.make(dev)

    dev = xtalx.modbus_adapter.find_one_mba()
    if dev is not None:
        bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=args.baud_rate)
        bus.set_vext(True)
        time.sleep(0.2)
        x = xtalx.p_sensor.XHTISM(bus, int(args.modbus_addr, 0))
        if args.serial_number is None or x.serial_num == args.serial_number:
            return x

    dev = xtalx.spi_adapter.find_one_spia()
    if dev is not None:
        bus = xtalx.spi_adapter.make_spia(dev)
        bus.set_vext(True)
        time.sleep(0.2)
        x = xtalx.p_sensor.XHTISS(bus)
        if args.serial_number is None or x.serial_num == args.serial_number:
            x.poll_interval_sec = 0.2
            return x

    raise Exception('No matching devices.')


def main(args):
    x = make_sensor(args)
    print('Found sensor: %s' % x.serial_num)
    print('  FW Version: %s' % x.fw_version_str)
    print('    Git SHA1: %s' % x.git_sha1)

    mt = threading.Thread(target=measure_thread, args=(x,))
    mt.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
    finally:
        x.halt_yield()
        mt.join()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--intf', '-i')
    parser.add_argument('--baud-rate', type=int, default=115200)
    parser.add_argument('--modbus-addr', '-m', default='0x80')
    parser.add_argument('--serial-number', '-s')
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
