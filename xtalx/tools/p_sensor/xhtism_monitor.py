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
import xtalx.modbus_adapter
import xtalx.tools.modbus.serial


# Verbosity
LOG_LEVEL = logging.INFO


def sensor_thread(xhtism, idb):
    # Monitor the sensor.
    logging.info('%s: Monitoring...', xhtism.serial_num)
    for m in xhtism.yield_measurements():
        if idb:
            point = m.to_influx_point()
            if point['fields']:
                idb.append(point)

        logging.info('%s: tf %.6f pf %.6f ',
                     xhtism.serial_num, m.temp_freq, m.pressure_freq)


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
    if rv.intf:
        bus = xtalx.tools.modbus.serial.Bus(rv.intf, rv.baud_rate)
    else:
        dev = xtalx.modbus_adapter.find_one_mba(
                serial_number=rv.modbus_adapter_serial_number)
        if dev is None:
            raise Exception('No adapter found.')
        bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=rv.baud_rate)
        bus.set_vext(True)
        time.sleep(0.1)

    # Spawn a thread for each sensor on the bus.
    xhtisms = []
    threads = []
    addrs = rv.addr
    if addrs is None:
        addrs = ['0x80']
    for addr in addrs:
        xhtism = xtalx.p_sensor.XHTISM(bus, int(addr, 0))
        logging.info('%s: Found sensor with firmware version %s, git SHA1 %s',
                     xhtism.serial_num, xhtism.fw_version_str, xhtism.git_sha1)
        t_c, p_c = xhtism.get_coefficients()
        logging.info('%s: T Coefficient: %u', xhtism.serial_num, t_c)
        logging.info('%s: P Coefficient: %u', xhtism.serial_num, p_c)
        xhtisms.append(xhtism)

        t = threading.Thread(target=sensor_thread, args=(xhtism, idb))
        t.start()
        threads.append(t)

    # Run forever.
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()

    # Halt and join all threads for a clean exit.
    for xhtism in xhtisms:
        xhtism.halt_yield()
    for t in threads:
        t.join()


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c')
    parser.add_argument('--intf', '-i')
    parser.add_argument('--baud-rate', '-b', default=115200, type=int)
    parser.add_argument('--addr', '-a', action='append', default=None)
    parser.add_argument('--modbus-adapter-serial-number', '-s')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
