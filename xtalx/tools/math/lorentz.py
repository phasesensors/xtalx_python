# Copyright (c) 2022-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import math

import scipy.optimize
import numpy as np


def lorentz_function(x, A, x0, W):
    '''
    Lorentz function with no background term and no Y offset.
    '''
    return (A / math.pi) * (W / ((x - x0)**2 + W**2))


class Lorentzian:
    def __init__(self, A, x0, W, RR=0, X=None, Y=None):
        if A < 0 and W < 0:
            A = -A
            W = -W

        self.A  = A
        self.x0 = x0
        self.W  = W
        if X is None or Y is None:
            self.RR = RR
        else:
            self.RR = self._compute_RR(X, Y)

    @staticmethod
    def from_x_y(X, Y, A, x0, w):
        cf   = scipy.optimize.curve_fit(lorentz_function, X, Y, p0=(A, x0, w))
        popt = cf[0]
        return Lorentzian(popt[0], popt[1], popt[2], X=X, Y=Y)

    def __call__(self, x):
        return lorentz_function(x, self.A, self.x0, self.W)

    def __repr__(self):
        return ('Lorentzian(A=%f, x0=%f, W=%f, RR=%f)' %
                (self.A, self.x0, self.W, self.RR))

    def _compute_RR(self, X, Y):
        Yf     = np.array([self(x) for x in X])
        ss_res = np.sum((Y - Yf)**2)
        ss_tot = np.sum((Y - np.mean(Y))**2)
        return 1 - (ss_res / ss_tot)
