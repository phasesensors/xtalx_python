# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import threading
import time

import simple_tsdb

import xtalx.z_sensor

from . import z_common


TELEMETRY_RUNNING = True


class Delegate(z_common.ZDelegate):
    def __init__(self, z_logger, tc, pq):
        super().__init__(z_logger)
        
        self.pq = pq

        if pq and isinstance(pq, simple_tsdb.PushQueue):
            self.stsdb_path = 'sensor_data/tincan_data/' + tc.serial_num
            self.stsdb_sweep_path = ('sensor_data/tincan_sweep_data/' +
                                     tc.serial_num)

    def sweep_callback(self, tc, pt, t0_ns, duration_ms, points, fw_fit,
                       hires, temp_freq, temp_c):
        super().sweep_callback(tc, pt, t0_ns, duration_ms, points, fw_fit,
                               hires, temp_freq, temp_c)

        if self.pq:
            if isinstance(self.pq, simple_tsdb.PushQueue):
                p = tc.make_stsdb_point(t0_ns, duration_ms, fw_fit, hires,
                                        temp_freq)
                self.pq.append(p, self.stsdb_path)
                ps = tc.make_stsdb_sweep_points(t0_ns, points)
                self.pq.append_list(ps, self.stsdb_sweep_path)
            else:
                p = self.make_influx_point(t0_ns, duration_ms, fw_fit, hires,
                                           temp_freq)
                self.pq.append(p)


def poll_telemetry(tc, zl, pq):
    # Discard the first telemetry packet since it will be stale.
    tc.read_telemetry()
    seq_num = None

    if pq:
        if isinstance(pq, simple_tsdb.PushQueue):
            path = 'sensor_data/tincan_temp_data/' + tc.serial_num
        else:
            pq = None

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
            if pq:
                pq.append(t.to_stsdb_point(), path)


def main(rv):
    global TELEMETRY_RUNNING

    c, pq = z_common.parse_config(rv)

    dev    = xtalx.z_sensor.find_one(serial_number=rv.sensor)
    tc     = xtalx.z_sensor.make(dev, verbose=rv.verbose,
                                 yield_Y=not rv.track_impedance)
    za, zl = z_common.parse_args(tc, rv)
    pt     = xtalx.z_sensor.PeakTracker(tc, za.amplitude, za.nfreqs,
                                        za.search_time_secs,
                                        za.sweep_time_secs,
                                        settle_ms=za.settle_ms,
                                        delegate=Delegate(zl, tc, pq))
    pt.start_threaded()

    telemetry_thread = None
    if rv.show_t_telemetry and tc.is_telemetry_supported():
        telemetry_thread = threading.Thread(target=poll_telemetry,
                                            args=(tc, zl, pq))
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
