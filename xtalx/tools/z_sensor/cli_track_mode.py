# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import time

import xtalx.z_sensor

from . import z_common


def main(rv):
    dev    = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc     = xtalx.z_sensor.make(dev, verbose=rv.verbose,
                                 yield_Y=not rv.track_impedance)
    za, zl = z_common.parse_args(tc, rv)
    pt     = xtalx.z_sensor.PeakTracker(tc, za.amplitude, za.nfreqs,
                                        za.search_time_secs,
                                        za.sweep_time_secs,
                                        settle_ms=za.settle_ms,
                                        delegate=z_common.ZDelegate(zl))
    pt.start_threaded()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
    finally:
        pt.stop_threaded()


def _main():
    parser = argparse.ArgumentParser()
    z_common.add_arguments(parser)
    rv = parser.parse_args()
    main(rv)


if __name__ == '__main__':
    _main()
