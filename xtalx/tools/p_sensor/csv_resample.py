import math
import argparse

from xtalx.tools.math import XYSeries
from xtalx.tools.csv import Decoder


CSV_FORMAT = {
    'time'          : float,
    'temp_c'        : float,
    'pressure_psi'  : float,
}


def resample(path, period):
    with open(path, 'r', encoding='utf8') as f:
        d = Decoder(f, CSV_FORMAT)

    X = [p['time'] - d.points[0]['time'] for p in d.points]
    t_series = XYSeries(X, [p['temp_c'] for p in d.points])
    p_series = XYSeries(X, [p['pressure_psi'] for p in d.points])
    N = math.ceil(X[-1] / period)

    rt_series = XYSeries([], [])
    rp_series = XYSeries([], [])
    for i in range(N):
        t0 = i*period
        rt_series.append(t0, t_series.get_avg_value(t0, t0+period))
        rp_series.append(t0, p_series.get_avg_value(t0, t0+period))

    return rt_series, rp_series


def main(args):
    rt_series, rp_series = resample(args.input_file, args.period)
    assert len(rt_series.X) == len(rp_series.X)

    with open(args.output_file, 'x', encoding='utf8') as f:
        f.write('time,temp_c,pressure_psi\n')
        for i, _ in enumerate(rt_series.X):
            f.write('%.6f,%.2f,%.6f\n' % (rt_series.X[i],
                                          rt_series.Y[i],
                                          rp_series.Y[i]))


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-file', '-i', required=True)
    parser.add_argument('--output-file', '-o', required=True)
    parser.add_argument('--period', '-p', type=float, default=1.0)
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
