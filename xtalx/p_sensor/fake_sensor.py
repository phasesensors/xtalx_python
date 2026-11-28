# Copyright (c) 2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time

import numpy
from numpy.polynomial import polynomial, polyutils


class Measurement:
    '''
    Object encapsulating a single FakeSensor measurement.  The following
    fields are defined:

        sensor - Reference to the FakeSensor that generated the Measurement.
        time_ns - Time of the measurement in nanoseconds since the epoch.
        pressure_psi - Pressure computed from the crystal frequencies using
            the sensor's pressure polynomial.
        temp_c - Temperature computed from the temperature crystal frequency
            using the sensor's temperature polynomial.
        pressure_freq - Pressure crystal frequency in Hz.
        temp_freq - Temperature crystal frequency in Hz.
    '''
    def __init__(self, sensor, time_ns, pressure_psi, temp_c, pressure_freq,
                 temp_freq):
        self.sensor        = sensor
        self.time_ns       = time_ns
        self.pressure_psi  = pressure_psi
        self.temp_c        = temp_c
        self.pressure_freq = pressure_freq
        self.temp_freq     = temp_freq


def _invert_poly(coefs, value, domain, window=(-1, 1)):
    '''
    Solves p(x) = value for x, where p is the polynomial with the given
    coefficients evaluated after mapping x linearly from domain onto window,
    as numpy.polynomial.Polynomial(coefs, domain, window) does.  If there are
    several real solutions, returns the one nearest the middle of the domain.
    Raises ValueError if there is no real solution.
    '''
    c     = numpy.array(coefs, dtype=float)
    c[0] -= value
    mid   = (window[0] + window[1]) / 2
    roots = sorted(polynomial.polyroots(c), key=lambda r: abs(r - mid))
    for r in roots:
        if abs(r.imag) <= 1e-6:
            u = r.real
            break
    else:
        raise ValueError('No real solution for %s.' % value)

    # Refine the root with a few Newton-Raphson iterations.
    dc = polynomial.polyder(c)
    for _ in range(3):
        d = polynomial.polyval(u, dc)
        if d == 0:
            break
        u -= polynomial.polyval(u, c) / d

    return float(polyutils.mapdomain(u, window, domain))


class FakeSensor:
    '''
    Simulated pressure sensor for testing tools without hardware.

    serial_num is the sensor's serial number string.  t_poly and p_poly are
    calibration polynomials in the same form as those used by
    CalPage.poly_psi and CalPage.poly_temp: t_poly(Ft) gives the
    temperature in degrees C and p_poly(Fp, Ft) gives the pressure in PSI,
    where Fp and Ft are the pressure and temperature crystal frequencies in
    Hz.  The crystal frequencies are taken from the pressure_freq_hz and
    temp_freq_hz fields, which the constructor computes from pressure_psi
    and temp_c with compute_freqs() and which may be changed at any time to
    simulate a change in pressure or temperature.
    '''
    def __init__(self, serial_num, t_poly, p_poly, pressure_psi=13.5,
                 temp_c=22):
        self.serial_num   = serial_num
        self.t_poly       = t_poly
        self.p_poly       = p_poly
        self.last_time_ns = 0
        (self.temp_freq_hz,
         self.pressure_freq_hz) = self.compute_freqs(temp_c, pressure_psi)

    def __str__(self):
        return 'FakeSensor(%s)' % self.serial_num

    def read_measurement(self):
        '''
        Returns a Measurement taken at the current crystal frequencies.
        '''
        fp = self.pressure_freq_hz
        ft = self.temp_freq_hz
        return Measurement(self, self.time_ns_increasing(),
                           self.p_poly(fp, ft), self.t_poly(ft), fp, ft)

    def compute_freqs(self, temp_c, pressure_psi):
        '''
        Inverts the calibration polynomials, returning the crystal
        frequencies (temp_freq, pressure_freq) in Hz for which t_poly gives
        temp_c and p_poly gives pressure_psi.  The temperature frequency is
        found first, then the pressure frequency at that temperature
        frequency.  Where a polynomial has several real solutions, the one
        nearest the middle of its calibration window is used.  Raises
        ValueError if there is no real solution.

        This relies on the internals of xtalx's PolynomialFit1D (a numpy
        Polynomial in pf) and PolynomialFit2D (x_domain, y_domain and a
        coefs matrix with coefs[i][j] the coefficient of x**i * y**j on the
        mapped [-1, 1] range).
        '''
        pf = self.t_poly.pf
        ft = _invert_poly(pf.coef, temp_c, pf.domain, pf.window)

        C  = numpy.asarray(self.p_poly.coefs)
        v  = polyutils.mapdomain(ft, self.p_poly.y_domain, (-1, 1))
        fp = _invert_poly(C @ v**numpy.arange(C.shape[1]), pressure_psi,
                          self.p_poly.x_domain)

        return ft, fp

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
