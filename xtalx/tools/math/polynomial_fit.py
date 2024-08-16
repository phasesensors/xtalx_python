# Copyright (c) 2020-2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import math
import numpy

import btype

from .hex_funcs import hex_to_double


TYPE_TABLE = {}
TYPE_TABLE_DOUBLE = {}


class PolyWindow(btype.Struct):
    low     = btype.uint64_t(0xFFFFFFFFFFFFFFFF)
    high    = btype.uint64_t(0xFFFFFFFFFFFFFFFF)


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


class PolynomialFit1D:
    def __init__(self, order, pf):
        self.order = order
        self.pf    = pf

    @staticmethod
    def from_domain_coefs(x_domain, coefs):
        '''
        Given the x_domain and a list of coefficients of the form:

            [x0, x1, x2, ..., xN]

        generate the fit polynomial for evaluation purposes.
        '''
        order = len(coefs) - 1
        pf    = numpy.polynomial.polynomial.Polynomial(coefs, domain=x_domain)
        return PolynomialFit1D(order, pf)

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
