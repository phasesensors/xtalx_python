import argparse

import glotlib
import numpy as np

from xtalx.tools import csv


CSV_FORMAT = {
    't'             : float,
    't_hz'          : float,
    'p_hz'          : float,
    'temp_c'        : float,
    'psi'           : float,
}


def main(args):
    with open(args.csv, 'r', encoding='utf8') as f:
        d = csv.Decoder(f, CSV_FORMAT)

    w = glotlib.Window(512, 512, msaa=4)
    p_plot = w.add_plot(211)
    t_plot = w.add_plot(212, sharex=p_plot)

    PRESSURE = np.array([p['psi'] for p in d.points])
    if any(p is None for p in PRESSURE):
        p_plot.set_y_label('Pressure (Hz)')
        PRESSURE = np.array([p['p_hz'] for p in d.points])
    else:
        p_plot.set_y_label('Pressure (PSI)')

    TEMP = np.array([p['temp_c'] for p in d.points])
    if any(t is None for t in TEMP):
        t_plot.set_y_label('Temp (Hz)')
        TEMP = np.array([p['t_hz'] for p in d.points])
    else:
        t_plot.set_y_label('Temp (C)')
    t_plot.set_x_label('Time (seconds)')

    X = np.array([p['t'] for p in d.points])

    p_plot.add_steps(X=X, Y=PRESSURE)
    t_plot.add_steps(X=X, Y=TEMP)
    p_plot.snap_bounds()
    t_plot.snap_bounds()

    try:
        glotlib.interact()
    except KeyboardInterrupt:
        print()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', required=True)
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
