# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse

import numpy as np

import xtalx.z_sensor
from xtalx.tools.z_sensor import z_common


def std_dev_predicate(measurements):
    std_f = np.std([m.peak_hz for m in measurements])
    return std_f < 0.001


def control_loop(pq, n_measurements):
    while True:
        measurements = pq.wait_predicate(std_dev_predicate, n_measurements)
        if measurements:
            print('Predicate matched!')
        else:
            print('Predicate timed out.')
        pq.clear()


def main(rv):
    dev    = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc     = xtalx.z_sensor.make(dev, verbose=rv.verbose,
                                 yield_Y=not rv.track_impedance)
    za, zl = z_common.parse_args(tc, rv)

    pq = xtalx.z_sensor.PredicateQueue(delegate=z_common.ZDelegate(zl))
    pt = xtalx.z_sensor.PeakTracker(tc, za.amplitude, za.nfreqs,
                                    za.search_time_secs, za.sweep_time_secs,
                                    settle_ms=za.settle_ms, delegate=pq)
    pt.start_threaded()

    try:
        control_loop(pq, rv.n_measurements)
    except KeyboardInterrupt:
        print()
    finally:
        pt.stop_threaded()


def _main():
    parser = argparse.ArgumentParser()
    z_common.add_arguments(parser)
    parser.add_argument('--n-measurements', type=int, default=5)
    rv = parser.parse_args()
    main(rv)


if __name__ == '__main__':
    _main()
