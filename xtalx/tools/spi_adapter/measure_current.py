# Copyright (c) 2026 Phase Advanced Sensor Systems, Inc.
import argparse
import threading
import time

import xtalx.spi_adapter


MONITORING_CURRENT = True


def make_spia(args):
    dev = xtalx.spi_adapter.find_one_spia(serial_number=args.serial_number)
    if dev is None:
        raise Exception('No SPIA found.')

    return xtalx.spi_adapter.make_spia(dev)


def current_monitor_thread(spia):
    print('Monitoring current consumption...')
    while MONITORING_CURRENT:
        amps = spia.measure_current()
        print('%.2f mA' % (amps * 1000))

        time.sleep(0.1)


def main(args):
    global MONITORING_CURRENT

    spia = make_spia(args)
    print('Found MBA: %s' % spia.serial_num)
    print('  FW SHA1: %s' % spia.git_sha1)
    print('FW Vesion: %s' % spia.fw_version_str)
    print('----------------------')

    print('Enabling VEXT...')
    spia.set_vext(True)

    cmt = threading.Thread(target=current_monitor_thread, args=(spia,))
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
