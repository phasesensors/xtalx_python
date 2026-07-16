# Copyright (c) 2026 by Phase Advanced Sensor Systems Corp.
import argparse

from xtalx.tools.math import PolynomialFit1D
import xtalx.t_sensor


# Sample temperature polynomial.
T_PF = PolynomialFit1D.from_domain_coefs(
    [261976.0054906831064727, 263922.5675630941987038],
    [
        111.4391012559056549,
        94.9733494361222057,
        -6.0488673984518551,
        0.0776806736368592,
        -0.3931375985579040,
    ])


def main(args):
    xtp = xtalx.t_sensor.make(xtalx.t_sensor.find_one())
    t_p = xtp.get_t_poly()
    if t_p is not None:
        print('Existing polynomial:')
        print(t_p)
    else:
        print('No polynomial present.')

    if args.erase_t_poly:
        print('Erasing polynomial...')
        xtp.set_t_poly(None)
        print('Success.')

    if args.write_t_poly:
        print('Updating polynomial...')
        xtp.set_t_poly(T_PF)
        print('Success.')


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--write-t-poly', action='store_true')
    parser.add_argument('--erase-t-poly', action='store_true')
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
