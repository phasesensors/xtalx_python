#!/usr/bin/env python3
# Copyright (c) 2020-2021 by Phase Advanced Sensor Systems Corp.
import argparse

import xtalx


def xtalx_cb(m):
    print('%s: C %u pe %u prc %u pf %f te %u trc %u tf %f p %s t %s mt %s'
          % (m.sensor, m.ref_freq, m.pressure_edges, m.pressure_ref_clocks,
             m.pressure_freq, m.temp_edges, m.temp_ref_clocks, m.temp_freq,
             m.pressure_psi, m.temp_c, m.mcu_temp_c))


def main(args):
    if args.serial_number is not None:
        sensors = xtalx.find(serial_number=args.serial_number)
        if not sensors:
            print('No matching sensors.')
            for s in xtalx.find():
                print('    %s' % s.serial_number)
            return
    else:
        sensors = xtalx.find()
        if not sensors:
            print('No sensors found.')
            return
    if len(sensors) != 1:
        print('Matching sensors:')
        for s in sensors:
            print('    %s' % s.serial_number)
        return
    d = sensors[0]

    x = xtalx.XtalX(d)
    x.read_measurements(xtalx_cb)
    try:
        x.join_read()
    except KeyboardInterrupt:
        x.halt_read()
        raise


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
