#!/usr/bin/env python3
# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
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


def make_sensor(rv):
    if rv.intf:
        bus = xtalx.tools.modbus.serial.Bus(rv.intf, rv.baud_rate)
        return xtalx.p_sensor.XHTISM(bus, int(rv.addr, 0))

    dev = xtalx.modbus_adapter.find_one_mba(
            serial_number=rv.modbus_adapter_serial_number)
    if dev is not None:
        bus = xtalx.modbus_adapter.make_mba(dev, baud_rate=rv.baud_rate)
        bus.set_vext(True)
        time.sleep(0.1)
        return xtalx.p_sensor.XHTISM(bus, int(rv.addr, 0))

    raise Exception('No matching devices.')


def main(rv):
    # Make the sensor.
    xhtism = make_sensor(rv)
    logging.info('%s: Found sensor with firmware version %s, git SHA1 %s',
                 xhtism.serial_num, xhtism.fw_version_str, xhtism.git_sha1)
    t_c, p_c = xhtism.get_coefficients()
    logging.info('%s: T Coefficient: %u', xhtism.serial_num, t_c)
    logging.info('%s: P Coefficient: %u', xhtism.serial_num, p_c)

    # Read the configuration file.
    if rv.config:
        logging.info('%s: Reading configuration...', xhtism.serial_num)
        with open(rv.config, encoding='utf8') as f:
            c = Config(f.readlines(), ['influx_host', 'influx_user',
                                       'influx_password', 'influx_database'])

        # Open a connection to InfluxDB.
        logging.info('%s: Connecting to InfluxDB...', xhtism.serial_num)
        idb = InfluxDBPushQueue(c.influx_host, 8086, c.influx_user,
                                c.influx_password, database=c.influx_database,
                                ssl=True, verify_ssl=True,
                                timeout=100, throttle_secs=10)
    else:
        idb = None

    # Monitor the sensor.
    logging.info('%s: Monitoring...', xhtism.serial_num)
    for m in xhtism.yield_measurements():
        if idb:
            point = m.to_influx_point()
            if point['fields']:
                idb.append(point)

        logging.info('%s: tf %.6f pf %.6f ',
                     xhtism.serial_num, m.temp_freq, m.pressure_freq)


def _main():
    logging.basicConfig(format='\033[1m[%(asctime)s.%(msecs)03d]\033[0m '
                        '%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    logging.getLogger().setLevel(LOG_LEVEL)

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c')
    parser.add_argument('--intf', '-i')
    parser.add_argument('--baud-rate', '-b', default=115200, type=int)
    parser.add_argument('--addr', '-a', default='0x80')
    parser.add_argument('--modbus-adapter-serial-number', '-s')

    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
