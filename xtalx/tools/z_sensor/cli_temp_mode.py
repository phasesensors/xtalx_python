# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import threading
import time

import xtalx.z_sensor


TEMP_RUNNING = True


def temp_thread(tc, interval_secs):
    tc.start_fixed_out(0, 0)
    tc.set_t_enable(True)

    t0_crystal_ticks, t0_cpu_ticks = tc.read_temp()
    while TEMP_RUNNING:
        time.sleep(interval_secs)
        t1_crystal_ticks, t1_cpu_ticks = tc.read_temp()

        dt = (t1_cpu_ticks - t0_cpu_ticks) / tc.CPU_FREQ
        dcrystal = (t1_crystal_ticks - t0_crystal_ticks) & 0xFFFFFFFF
        t0_crystal_ticks, t0_cpu_ticks = t1_crystal_ticks, t1_cpu_ticks

        if dt == 0:
            tc.warn('Temp crystal does not appear to be ticking.')
            continue

        temp_hz = dcrystal * 8 / dt

        temp_c, _, _ = tc.eval_freqs(temp_hz, 0, 0)
        tc.info('temp_hz %s T %s' % (temp_hz, temp_c))


def main(args):
    global TEMP_RUNNING

    dev = xtalx.z_sensor.find_one(serial_number=args.sensor)
    tc  = xtalx.z_sensor.make(dev, verbose=args.verbose)
    t   = threading.Thread(target=temp_thread, args=(tc, args.interval_secs))
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
    finally:
        TEMP_RUNNING = False
        t.join()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', '-s')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--interval-secs', '-i', type=float, default=0.1)
    args = parser.parse_args()
    main(args)


if __name__ == '__main__':
    _main()
