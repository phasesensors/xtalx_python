#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import logging
import time

import xtalx.p_sensor
import xtalx.modbus_adapter
import xtalx.tools.modbus.serial


# Verbosity
LOG_LEVEL = logging.INFO


def make_sensor(args):
    if args.intf:
        bus = xtalx.tools.modbus.serial.Bus(args.intf, args.baud_rate,
                                            parity=args.parity)
        return xtalx.p_sensor.XHTISM(bus, int(args.addr, 0))

    dev = xtalx.modbus_adapter.find_one_mba(
            serial_number=args.modbus_adapter_serial_number)
    if dev is not None:
        bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=args.baud_rate,
                                            parity=args.parity)
        bus.set_vext(True)
        time.sleep(0.1)
        return xtalx.p_sensor.XHTISM(bus, int(args.addr, 0))

    raise Exception('No matching devices.')


def main(args):
    # Make the sensor.
    xhtism = make_sensor(args)
    logging.info('%s: Found sensor with firmware version %s, git SHA1 %s',
                 xhtism.serial_num, xhtism.fw_version_str, xhtism.git_sha1)
    t_c, p_c = xhtism.get_coefficients()
    logging.info('%s: T Coefficient: %u', xhtism.serial_num, t_c)
    logging.info('%s: P Coefficient: %u', xhtism.serial_num, p_c)

    # If we need to update the T/P coefficients, do so now.
    if args.set_t_coefficient or args.set_p_coefficient:
        if args.set_t_coefficient:
            t_c = int(args.set_t_coefficient, 0)
        if args.set_p_coefficient:
            p_c = int(args.set_p_coefficient, 0)
        logging.info('%s: Updating coefficients...', xhtism.serial_num)
        xhtism.set_coefficients(t_c, p_c)

    # If we need to change the comms params, now is the time.
    if args.set_addr or args.set_baud_rate or args.set_parity:
        logging.info('%s: Updating comm params...', xhtism.serial_num)
        if args.set_baud_rate:
            new_baud_rate = args.set_baud_rate
        else:
            new_baud_rate = args.baud_rate
        if args.set_parity:
            new_parity = args.set_parity
        else:
            new_parity = args.parity
        if args.set_addr:
            addr = int(args.set_addr, 0)
        else:
            addr = xhtism.slave_addr
        xhtism.set_comm_params(new_baud_rate, addr, new_parity)


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--intf', '-i')
    parser.add_argument('--baud-rate', '-b', default=115200, type=int)
    parser.add_argument('--parity', default='E', choices=['E', 'O', 'N'])
    parser.add_argument('--addr', '-a', default='0x80')
    parser.add_argument('--set-addr')
    parser.add_argument('--set-baud-rate', type=int)
    parser.add_argument('--set-parity', choices=['E', 'O', 'N'])
    parser.add_argument('--set-t-coefficient')
    parser.add_argument('--set-p-coefficient')
    parser.add_argument('--modbus-adapter-serial-number', '-s')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
