#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import argparse
import logging
import time

from xtalx.tools.config import Config
from xtalx.tools.influxdb import InfluxDBPushQueue
import xtalx.p_sensor
import xtalx.spi_adapter


# Verbosity
LOG_LEVEL = logging.INFO


def sensor_thread(xhtiss, idb):
    # Monitor the sensor.
    logging.info('%s: Monitoring...', xhtiss.serial_num)
    for m in xhtiss.yield_measurements(poll_interval_sec=0.05):
        if idb:
            point = m.to_influx_point()
            if point['fields']:
                idb.append(point)

        logging.info('%s: t %.3f p %.3f tf %.6f pf %.6f ',
                     xhtiss.serial_num, m.temp_c, m.pressure_psi,
                     m.temp_freq, m.pressure_freq)


def main(rv):
    # Read the configuration file.
    if rv.config:
        logging.info('Reading configuration...')
        with open(rv.config, encoding='utf8') as f:
            c = Config(f.readlines(), ['influx_host', 'influx_user',
                                       'influx_password', 'influx_database'])

        # Open a connection to InfluxDB.
        logging.info('Connecting to InfluxDB...')
        idb = InfluxDBPushQueue(c.influx_host, 8086, c.influx_user,
                                c.influx_password, database=c.influx_database,
                                ssl=True, verify_ssl=True,
                                timeout=100, throttle_secs=10)
    else:
        idb = None

    # Make the bus.
    dev = xtalx.spi_adapter.find_one_spia(
            serial_number=rv.adapter_serial_number)
    if dev is None:
        raise Exception('No adapter found.')
    bus = xtalx.spi_adapter.make_spia(dev)
    bus.set_vext(True)
    time.sleep(0.1)

    # Spawn a thread for each sensor on the bus.
    xhtiss = xtalx.p_sensor.XHTISS(bus)
    logging.info('%s: Found sensor with firmware version %s, git SHA1 %s',
                 xhtiss.serial_num, xhtiss.fw_version_str, xhtiss.git_sha1)
    t_c, p_c, sample_ms = xhtiss.get_flash_params()
    logging.info('%s: T Coefficient: %u', xhtiss.serial_num, t_c)
    logging.info('%s: P Coefficient: %u', xhtiss.serial_num, p_c)
    logging.info('%s: Sample Interval: %u ms', xhtiss.serial_num, sample_ms)

    t = threading.Thread(target=sensor_thread, args=(xhtiss, idb))
    t.start()

    # Run forever.
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()

    # Halt and join all threads for a clean exit.
    xhtiss.halt_yield()
    t.join()


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c')
    parser.add_argument('--adapter-serial-number', '-s')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
