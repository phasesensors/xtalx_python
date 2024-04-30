#!/usr/bin/env python3
# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import time

import xtalx.z_sensor


def main(rv):
    dev = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc  = xtalx.z_sensor.make(dev)
    tc.set_t_enable(True)
    time.sleep(0.5)
    while True:
        t0_crystal_ticks, t0_cpu_ticks = tc.read_temp()
        time.sleep(0.5)
        t1_crystal_ticks, t1_cpu_ticks = tc.read_temp()

        dt = (t1_cpu_ticks - t0_cpu_ticks) / tc.CPU_FREQ
        if dt == 0:
            print('Temp crystal not ticking!')
            continue

        dcrystal = (t1_crystal_ticks - t0_crystal_ticks) & 0xFFFFFFFF
        print('T Freq: %s Hz' % (dcrystal * 8 / dt))


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor')
    rv = parser.parse_args()

    try:
        main(rv)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
