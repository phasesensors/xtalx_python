#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import logging
import time

import simple_tsdb

import xtalx.p_sensor


# Verbosity
LOG_LEVEL = logging.INFO

MONITORING = True


def sensor_thread(x, pq):
    global MONITORING

    if pq:
        path = 'sensor_data/xtalx_data/' + x.serial_num

    try:
        # Monitor the sensor.
        logging.info('%s: Found sensor with firmware version %s, git SHA1 %s',
                     x.serial_num, x.fw_version_str, x.git_sha1)
        x.read_measurement()
        while MONITORING:
            m = x.read_measurement()
            if pq:
                pq.append(m.to_stsdb_point(), path)
            logging.info('%s: t %.3f p %.3f tf %.6f pf %.6f ',
                         x.serial_num, m.temp_c, m.pressure_psi,
                         m.temp_freq, m.pressure_freq)
    finally:
        MONITORING = False


def main(rv):
    global MONITORING

    pq = None
    if rv.use_simple_tsdb:
        pq = simple_tsdb.PushQueue('127.0.0.1', 4000)

    # Make the bus.
    dev = xtalx.p_sensor.find_one_xti15(serial_number=rv.serial_number)
    if dev is None:
        raise Exception('No sensor found.')
    x = xtalx.p_sensor.make_xti15(dev)

    # Spawn a thread for the sensor.
    t = threading.Thread(target=sensor_thread, args=(x, pq))
    t.start()

    # Run forever.
    try:
        while MONITORING:
            time.sleep(1)
    except KeyboardInterrupt:
        print()

    # Halt and join thread for a clean exit.
    MONITORING = False
    t.join()


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--use-simple-tsdb', action='store_true')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
