#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import logging

import xtalx.p_sensor


# Verbosity
LOG_LEVEL = logging.INFO


def main(args):
    # Make the sensor.
    xhtism = xtalx.p_sensor.XHTISM(args.intf, args.baud_rate, int(args.addr, 0))
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
    if args.set_addr or args.set_baud_rate:
        logging.info('%s: Updating comm params...', xhtism.serial_num)
        if args.set_addr:
            addr = int(args.set_addr, 0)
        else:
            addr = xhtism.slave_addr
        xhtism.set_comm_params(args.set_baud_rate, addr)


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--intf', '-i', required=True)
    parser.add_argument('--baud-rate', '-b', default=115200, type=int)
    parser.add_argument('--addr', '-a', default='0x80')
    parser.add_argument('--set-addr')
    parser.add_argument('--set-baud-rate', type=int)
    parser.add_argument('--set-t-coefficient')
    parser.add_argument('--set-p-coefficient')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
