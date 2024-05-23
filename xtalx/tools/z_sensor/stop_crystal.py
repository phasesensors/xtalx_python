# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse

import xtalx.z_sensor


def main(args):
    dev = xtalx.z_sensor.find_one(serial_number=args.sensor)
    tc  = xtalx.z_sensor.make(dev, yield_Y=True)
    tc.info('Stopping sensor...')
    tc.send_scope_cmd(32768, 0)
    tc.info('Stopped.')


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', '-s')
    args = parser.parse_args()
    main(args)


if __name__ == '__main__':
    _main()
