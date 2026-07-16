#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse

import xtalx.p_sensor


def main(args):
    # Make the bus.
    dev = xtalx.p_sensor.find_one_xti15(serial_number=args.serial_number)
    if dev is None:
        raise Exception('No sensor found.')
    x = xtalx.p_sensor.make_xti15(dev)

    # Get the current settings.
    lhp_pid_params = x.get_lhp_pid_params()
    print('Current settings:')
    print('   Setpoint: %.12f Hz' % lhp_pid_params.lhp_setpoint_hz)
    print('         kP: %.12f Hz' % lhp_pid_params.lhp_pid_kp)
    print('         kI: %.12f Hz' % lhp_pid_params.lhp_pid_ki)
    print('         kD: %.12f Hz' % lhp_pid_params.lhp_pid_kd)
    print('   PID Type: %s' % lhp_pid_params.lhp_pid_type)

    if (args.setpoint_hz, args.p, args.i, args.d) == (None, None, None, None):
        return


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--setpoint-hz', type=float)
    parser.add_argument('-p', type=float)
    parser.add_argument('-i', type=float)
    parser.add_argument('-d', type=float)
    parser.add_argument('--save-params', action='store_true')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
