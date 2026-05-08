# Copyright (c) 2026 by Phase Advanced Sensor Systems, Inc.
import argparse
import threading
import time

import simple_tsdb

import xtalx.t_sensor


MONITORING = True


def telemetry_thread(xtp, sdb):
    # Discard the first measurement since it will be out-of-date.
    xtp.read_measurement(timeout=10000)

    path = 'sensor_data/xtp_data/' + xtp.serial_num
    while MONITORING:
        m = xtp.read_measurement(timeout=10000)
        if sdb:
            p = m.to_stsdb_point()
            sdb.append(p, path)

        xtp.log('M', m.tostring(True), timestamp=m.time_ns)


def main(args):
    global MONITORING

    if args.use_simple_tsdb:
        sdb = simple_tsdb.PushQueue('127.0.0.1', 4000)
    else:
        sdb = None

    if args.serial_number:
        dev = xtalx.t_sensor.find_one(serial_number=args.serial_number)
        if dev is None:
            print('Device %s not found.' % args.serial_number)
            return
    else:
        dev = xtalx.t_sensor.find_one()
        if dev is None:
            print('No devices found.')
            return
    xtp = xtalx.t_sensor.make(dev)
    xtp.log('I', 'Monitoring sensor with firmware %s (%s)' %
            (xtp.fw_version_str, xtp.git_sha1))

    if args.sample_interval_ms:
        if args.sample_interval_ms <= 0:
            print('Invalid sample interval %d.' % args.sample_interval_ms)
            return
        xtp.set_sample_interval_ms(args.sample_interval_ms)

    if args.alpha_shift:
        if args.alpha_shift < 0 or args.alpha_shift > 20:
            print('Invalid alpha shift %d.' % args.alpha_shift)
            return
        xtp.set_iir_alpha_shift(args.alpha_shift)

    th = threading.Thread(target=telemetry_thread, args=(xtp, sdb))
    th.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
    finally:
        MONITORING = False
        th.join()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--use-simple-tsdb', action='store_true')
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--sample-interval-ms', type=int)
    parser.add_argument('--alpha-shift', type=int)
    args = parser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
