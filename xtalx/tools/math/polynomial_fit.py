# Copyright (c) 2020-2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import math
import numpy

import btype

from .hex_funcs import hex_to_double


SUPERSCRIPT = [
    '\u2070',
    '\u00B9',
    '\u00B2',
    '\u00B3',
    '\u2074',
    '\u2075',
    '\u2076',
    '\u2077',
    '\u2078',
    '\u2079',
    ]

SUBSCRIPT = [
    '\u2080',
    '\u2081',
    '\u2082',
    '\u2083',
    '\u2084',
    '\u2085',
    '\u2086',
    '\u2087',
    '\u2088',
    '\u2089',
    ]

TYPE_TABLE = {}
TYPE_TABLE_DOUBLE = {}


def number_script(v, script_array):
    assert isinstance(v, int)

    v = str(v)
    s = ''
    for c in v:
        s += script_array[ord(c) - 48]
    return s


def superscript(v):
    if v == 1:
        return ''
    return number_script(v, SUPERSCRIPT)


def subscript(v):
    return number_script(v, SUBSCRIPT)


class PolyWindow(btype.Struct):
    low     = btype.uint64_t(0xFFFFFFFFFFFFFFFF)
    high    = btype.uint64_t(0xFFFFFFFFFFFFFFFF)


class PolyWindowDouble(btype.Struct):
    low     = btype.float64_t()
    high    = btype.float64_t()


def make_poly_bstruct(Nvars, Ncoefs):
    if (Nvars, Ncoefs) in TYPE_TABLE:
        return TYPE_TABLE[(Nvars, Ncoefs)]

    class _Poly(btype.Struct):
        nvars   = btype.uint32_t(0xFFFFFFFF)
        order   = btype.uint32_t(0xFFFFFFFF)
        windows = btype.Array(PolyWindow(), Nvars)
        coefs   = btype.Array(btype.uint64_t(0xFFFFFFFFFFFFFFFF), Ncoefs)
    TYPE_TABLE[(Nvars, Ncoefs)] = _Poly

    return _Poly


def make_poly_bstruct_double(Nvars, Ncoefs):
    if (Nvars, Ncoefs) in TYPE_TABLE_DOUBLE:
        return TYPE_TABLE_DOUBLE[(Nvars, Ncoefs)]

    class _Poly(btype.Struct):
        nvars   = btype.uint32_t()
        order   = btype.uint32_t()
        windows = btype.Array(PolyWindowDouble(), Nvars)
        coefs   = btype.Array(btype.float64_t(), Ncoefs)
    TYPE_TABLE_DOUBLE[(Nvars, Ncoefs)] = _Poly

    return _Poly


class PolynomialFit1D:
    def __init__(self, order, pf, RR=None):
        self.order = order
        self.pf    = pf
        self.RR    = RR

    @staticmethod
    def from_samples(X, Y, order=1):
        '''
        Given a set of (x, y) samples, generate the best least-squares fit to
        the data using the given order.
        '''
        assert len(X) == len(Y)
        X      = numpy.array(X)
        Y      = numpy.array(Y)
        pf     = numpy.polynomial.polynomial.Polynomial.fit(X, Y, order)
        y_mean = numpy.mean(Y)
        SStot  = numpy.sum((Y - y_mean)**2)
        SSreg  = numpy.sum((pf(X) - y_mean)**2)
        RR     = SSreg / SStot if SStot != 0 else None
        return PolynomialFit1D(order, pf, RR=RR)

    @staticmethod
    def from_domain_coefs(x_domain, coefs):
        '''
        Given the x_domain and a list of coefficients of the form:

            x_domain = [low, high]
            coefs    = [x0, x1, x2, ..., xN]

        generate the fit polynomial for evaluation purposes.
        '''
        order = len(coefs) - 1
        pf    = numpy.polynomial.polynomial.Polynomial(coefs, domain=x_domain)
        return PolynomialFit1D(order, pf)

    @staticmethod
    def from_k1_k2_coefs(k1, k2, coefs):
        '''
        Given k1 and k2 and a list of coefficients of the form:

            k1    = 2 / (high - low)
            k2    = low * k1 + 1
            coefs = [x0, x1, x2, ..., xN]

        generate the fit polynomial for evaluation purposes.
        '''
        low   = (k2 - 1) / k1
        high  = low + 2 / k1
        return PolynomialFit1D.from_domain_coefs([low, high], coefs)

    @staticmethod
    def from_poly_bstruct(p):
        '''
        Generate the fit polynomial from a make_poly_bstruct() btype class.
        '''
        assert p.nvars == 1
        order  = p.order
        x_domain = (hex_to_double(p.windows[0].low),
                    hex_to_double(p.windows[0].high))
        coefs = [hex_to_double(p.coefs[i]) for i in range(order + 1)]
        return PolynomialFit1D.from_domain_coefs(x_domain, coefs)

    def __call__(self, x):
        return self.pf(x)

    @staticmethod
    def _polystr(coef):
        s      = '%.15f' % coef[0]
        for i, c in enumerate(coef[1:]):
            s += ' + %.15f*x%s' % (c, superscript(i + 1))
        return s

    def __repr__(self):
        s = PolynomialFit1D._polystr(self.pf.coef)
        s += '\n : x(%s, %s) -> [-1. 1.]' % (float(self.pf.domain[0]),
                                             float(self.pf.domain[1]))
        return s

    def to_poly_class_double_efficient(self, Ncoefs):
        cls               = make_poly_bstruct_double(1, Ncoefs)
        p                 = cls()
        k                 = 1 / (self.pf.domain[1] - self.pf.domain[0])
        p.nvars           = 1
        p.order           = self.order
        p.windows[0].low  = 2 * k
        p.windows[0].high = 2 * k * self.pf.domain[0] + 1
        for i, c in enumerate(self.pf.coef):
            p.coefs[i] = c
        return p


class PolynomialFit2D:
    def __init__(self, x_domain, y_domain, coefs):
        self.x_domain = x_domain
        self.y_domain = y_domain
        self.coefs    = coefs

    @staticmethod
    def from_domain_coefs(x_domain, y_domain, coefs):
        '''
        Given the x_domain, y_domain and a list of coefficients of the form:

            [x0y0, x1y0, x2y0, ..., xNy0,
             x0y1, x1y1, x2y1, ..., xNy1,
             ...
             x0yN, x1yN, x2yN, ..., xNyN]

        generate the fit polynomial for evaluation purposes.
        '''
        order = round(math.sqrt(len(coefs))) - 1
        return PolynomialFit2D(x_domain, y_domain,
                               numpy.reshape(coefs, (order + 1, order + 1)))

    @staticmethod
    def from_poly_bstruct(p):
        '''
        Generate the fit polynomial from a make_poly_bstruct() btype class.
        '''
        assert p.nvars == 2
        order = p.order
        x_domain = (hex_to_double(p.windows[0].low),
                    hex_to_double(p.windows[0].high))
        y_domain = (hex_to_double(p.windows[1].low),
                    hex_to_double(p.windows[1].high))
        coefs = [hex_to_double(p.coefs[i]) for i in range((order + 1)**2)]
        return PolynomialFit2D.from_domain_coefs(x_domain, y_domain, coefs)

    @staticmethod
    def _mapped(p, domain):
        l = domain[0]
        h = domain[1]
        return 2*(p - l)/(h - l) - 1

    def __call__(self, x, y):
        x = self._mapped(x, self.x_domain)
        y = self._mapped(y, self.y_domain)
        return numpy.polynomial.polynomial.polyval2d(x, y, self.coefs)

    @staticmethod
    def _polystr(coef):
        deg_x, deg_y = coef.shape
        s = '   |'
        for x in range(deg_x):
            if x == 0:
                p = '1'
            else:
                p = 'x%s' % superscript(x)
            s += ' %12s' % p
        l = '---+'
        for x in range(deg_x):
            l += ' ------------'
        s += '\n%s\n' % l
        for y in range(deg_y):
            if y == 0:
                p = '1'
            else:
                p = 'y%s' % superscript(y)
            s += '%-2s |' % p
            for x in range(deg_x):
                s += ' %12.5f' % coef[x][y]
            s += '\n'
        s += l
        return s

    def __repr__(self):
        return PolynomialFit2D._polystr(self.coefs) + (
            '\n : x%s -> [-1. 1.]\n : y%s -> [-1. 1.]' %
            (self.x_domain, self.y_domain))
