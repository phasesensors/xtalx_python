# Copyright (c) 2026 by Phase Advanced Sensor Systems Corp.
import argparse

from xtalx.tools.math import PolynomialFit1D
import xtalx.t_sensor


# Sample temperature polynomial.
T_PF = PolynomialFit1D.from_k1_k2_coefs(
    0.002313681848474984,
    607.1262361524482,
    [
        56.27999535663069,
        45.02740279604612,
        -1.2945479608644326,
        -0.029173760155661317,
        0.012404187667028355,
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
