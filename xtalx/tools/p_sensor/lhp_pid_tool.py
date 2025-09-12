#!/usr/bin/env python3
# Copyright (c) 2020-2024 by Phase Advanced Sensor Systems Corp.
import argparse

import xtalx.p_sensor


def main(args):
    d = xtalx.p_sensor.find_one_xti(serial_number=args.serial_number)
    x = xtalx.p_sensor.make_xti(d)
    if args.disable_pid:
        print('Disabling PID control...')
        x._disable_pid()
    if (args.p is not None or args.i is not None or args.d is not None or
            args.setpoint_c is not None):
        print('Setting PID coefficients and setpoint...')
        x._set_pid_info(Kp=args.p, Ki=args.i, Kd=args.d,
                        setpoint_c=args.setpoint_c)

    pid_info = x._get_pid_info()
    print('Current PID configuration:')
    print('   P        = %s' % pid_info.Kp)
    print('   I        = %s' % pid_info.Ki)
    print('   D        = %s' % pid_info.Kd)
    print('   Setpoint = %s C' % pid_info.setpoint_c)

    if args.enable_pid:
        print('Enabling PID control...')
        x._enable_pid()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--enable-pid')
    parser.add_argument('--disable-pid')
    parser.add_argument('-p', type=float)
    parser.add_argument('-i', type=float)
    parser.add_argument('-d', type=float)
    parser.add_argument('--setpoint-c', type=float)
    parser.add_argument('--verbose', '-v', action='store_true')
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
