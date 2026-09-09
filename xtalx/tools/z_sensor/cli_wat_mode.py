# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import time

import simple_tsdb

import xtalx.z_sensor
from xtalx.z_sensor.peak_tracker import Delegate


# Search parameters.
PT_NFREQS          = 20
PT_SEARCH_TIME_SEC = 5
ENABLE_CHIRP       = False

# Sweep parameters.
PT_SWEEP_TIME_SEC  = 5
PT_SETTLE_TIME_MS  = 5000


class WATDelegate(Delegate):
    def __init__(self, tc, pq, csv):
        self.tc             = tc
        self.pq             = pq
        self.csv            = csv

        if pq and isinstance(pq, simple_tsdb.PushQueue):
            self.stsdb_path = 'sensor_data/tincan_data/' + tc.serial_num
            self.stsdb_sweep_path = ('sensor_data/tincan_sweep_data/' +
                                     tc.serial_num)

    def sweep_callback(self, tc, pt, t0_ns, duration_ms, points, fw_fit,
                       hires, temp_freq, temp_c):
        tag = 'H' if hires else 'S'
        f0 = points[0].f
        f1 = points[-1].f
        tc.log(tag, 'i %u f0 %f f1 %f temp_hz %s T %s' %
               (pt.sweep, f0, f1, temp_freq, temp_c))

        if pt.sweep_iter <= 1:
            return

        if self.csv:
            time_ns = t0_ns + duration_ms * 1000
            dtime_ns = time_ns - pt.start_time_ns
            self.csv.write('%u,%u,%.3f,%.3f\n' %
                           (time_ns, dtime_ns, fw_fit.temp_c, fw_fit.peak_hz))
            self.csv.flush()

        if self.pq:
            p = self.tc.make_stsdb_point(t0_ns, duration_ms, fw_fit, hires,
                                         temp_freq)
            self.pq.append(p, self.stsdb_path)
            ps = self.tc.make_stsdb_sweep_points(t0_ns, points)
            self.pq.append_list(ps, self.stsdb_sweep_path)


def main(args, delegate_class):
    if args.use_simple_tsdb:
        pq = simple_tsdb.PushQueue('127.0.0.1', 4000)
    else:
        pq = None

    if args.output_csv:
        csv = open(  # pylint: disable=R1732
                args.output_csv, 'a', encoding='utf8')
        csv.write('time_ns,dt,temp_c,peak_hz\n')
        csv.flush()
    else:
        csv = None

    dev    = xtalx.z_sensor.find_one(serial_number=args.sensor)
    tc     = xtalx.z_sensor.make(dev, yield_Y=True)
    ww     = delegate_class(tc, pq, csv)
    a      = tc.parse_amplitude(None)
    pt     = xtalx.z_sensor.PeakTracker(tc, a, PT_NFREQS, PT_SEARCH_TIME_SEC,
                                        PT_SWEEP_TIME_SEC,
                                        settle_ms=PT_SETTLE_TIME_MS,
                                        delegate=ww, enable_chirp=ENABLE_CHIRP)
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
    parser.add_argument('--output-csv', '-o')
    parser.add_argument('--sensor', '-s')
    parser.add_argument('--use-simple-tsdb', action='store_true')
    args = parser.parse_args()
    main(args, WATDelegate)


if __name__ == '__main__':
    _main()
