# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import threading
import time

import xtalx.z_sensor

from . import z_common


TELEMETRY_RUNNING = True


def poll_telemetry(tc, zl):
    # Discard the first telemetry packet since it will be stale.
    tc.read_telemetry()
    seq_num = None

    while TELEMETRY_RUNNING:
        t = tc.read_telemetry()
        if not t:
            continue

        if seq_num is None:
            seq_num = t.seq_num
        elif t.seq_num != seq_num + 1:
            lost_seqs = t.seq_num - seq_num - 1
            tc.info('Lost %u telemetry packets.' % lost_seqs)
        seq_num = t.seq_num

        if isinstance(t, xtalx.z_sensor.tcsc_types.TelemetryTemperature):
            zl.log_telemetry_temp(tc, t)


def main(rv):
    global TELEMETRY_RUNNING

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

    telemetry_thread = None
    if rv.show_t_telemetry and tc.is_telemetry_supported():
        telemetry_thread = threading.Thread(target=poll_telemetry,
                                            args=(tc, zl))
        telemetry_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
    finally:
        TELEMETRY_RUNNING = False
        pt.stop_threaded()
        if telemetry_thread:
            telemetry_thread.join()


def _main():
    parser = argparse.ArgumentParser()
    z_common.add_arguments(parser)
    rv = parser.parse_args()
    main(rv)


if __name__ == '__main__':
    _main()
