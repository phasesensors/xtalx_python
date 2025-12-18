# Copyright (c) 2025 Phase Advanced Sensor Systems, Inc.
import argparse
import threading
import time

import xtalx.modbus_adapter


MONITORING_CURRENT = True


def make_mba(args):
    dev = xtalx.modbus_adapter.find_one_mba(serial_number=args.serial_number)
    if dev is None:
        raise Exception('No MBA found.')

    return xtalx.modbus_adapter.make_mba(dev)


def current_monitor_thread(mba):
    print('Monitoring current consumption...')
    while MONITORING_CURRENT:
        amps = mba.measure_current()
        print('%.2f mA' % (amps * 1000))

        time.sleep(0.1)


def main(args):
    global MONITORING_CURRENT

    mba = make_mba(args)
    print('Found MBA: %s' % mba.serial_num)
    print('  FW SHA1: %s' % mba.git_sha1)
    print('FW Vesion: %s' % mba.fw_version_str)
    print('----------------------')

    print('Enabling VEXT...')
    mba.set_vext(True)

    cmt = threading.Thread(target=current_monitor_thread, args=(mba,))
    cmt.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        MONITORING_CURRENT = False

    cmt.join()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
