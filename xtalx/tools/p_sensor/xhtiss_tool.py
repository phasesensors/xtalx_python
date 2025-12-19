#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import logging
import time

import xtalx.p_sensor
import xtalx.spi_adapter


# Verbosity
LOG_LEVEL = logging.INFO


def make_sensor(args):
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
    # Make the sensor.
    xhtiss = make_sensor(args)
    logging.info('%s: Found sensor with firmware version %s, git SHA1 %s',
                 xhtiss.serial_num, xhtiss.fw_version_str, xhtiss.git_sha1)
    t_c, p_c, sample_ms = xhtiss.get_flash_params()
    logging.info('%s: T Coefficient: %u', xhtiss.serial_num, t_c)
    logging.info('%s: P Coefficient: %u', xhtiss.serial_num, p_c)
    logging.info('%s:   Sample Rate: %u ms', xhtiss.serial_num, sample_ms)

    # If we need to update the T/P coefficients, do so now.
    if args.set_t_coefficient or args.set_p_coefficient:
        if args.set_t_coefficient:
            t_c = int(args.set_t_coefficient, 0)
        if args.set_p_coefficient:
            p_c = int(args.set_p_coefficient, 0)
        if args.set_sample_ms:
            sample_ms = int(args.set_sample_ms, 0)
        logging.info('%s: Updating coefficients...', xhtiss.serial_num)
        xhtiss.set_flash_params(t_c, p_c, sample_ms)


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--intf', '-i')
    parser.add_argument('--set-t-coefficient')
    parser.add_argument('--set-p-coefficient')
    parser.add_argument('--set-sample-ms')
    parser.add_argument('--serial-number', '-s')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
