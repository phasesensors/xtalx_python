# Copyright (c) 2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import time

import numpy
from numpy.polynomial import polynomial, polyutils


# Ambient conditions: the default readings, and the point where the
# simulated measurement error is just its offset.
AMBIENT_PSI = 13.5
AMBIENT_C   = 22


class Measurement:
    '''
    Object encapsulating a single FakeSensor measurement.  The following
    fields are defined:

        sensor - Reference to the FakeSensor that generated the Measurement.
        time_ns - Time of the measurement in nanoseconds since the epoch.
        pressure_psi - Pressure computed from the crystal frequencies using
            the sensor's pressure polynomial, plus the sensor's simulated
            pressure error.
        temp_c - Temperature computed from the temperature crystal frequency
            using the sensor's temperature polynomial, plus the sensor's
            simulated temperature error.
        pressure_freq - Pressure crystal frequency in Hz.
        temp_freq - Temperature crystal frequency in Hz.

    The frequencies are the ones that give pressure_psi and temp_c through the
    sensor's polynomials, so they include the simulated errors as well.
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
    and temp_c with compute_freqs().  To simulate a change in pressure or
    temperature, call set_temp_pressure(), which may be called from another
    thread while measurements are being read, or set the fields directly.

    To make sensors that measure the same conditions disagree the way real
    ones do, each measurement can include a simulated error that varies
    linearly with the reading, relative to ambient conditions (AMBIENT_PSI
    and AMBIENT_C):

        pressure error = p_error_offset_psi +
                         p_error_slope * (pressure - AMBIENT_PSI)
        temperature error = t_error_offset_c +
                            t_error_slope * (temperature - AMBIENT_C)

    where pressure and temperature are the polynomial values.  For example,
    p_error_slope=1e-4 gives an error of about 1.5 PSI at 15000 PSI.  All of
    the error terms default to 0.
    '''
    def __init__(self, serial_num, t_poly, p_poly, pressure_psi=AMBIENT_PSI,
                 temp_c=AMBIENT_C, *, p_error_offset_psi=0, p_error_slope=0,
                 t_error_offset_c=0, t_error_slope=0):
        self.serial_num         = serial_num
        self.t_poly             = t_poly
        self.p_poly             = p_poly
        self.p_error_offset_psi = p_error_offset_psi
        self.p_error_slope      = p_error_slope
        self.t_error_offset_c   = t_error_offset_c
        self.t_error_slope      = t_error_slope
        self.last_time_ns       = 0
        self.lock               = threading.Lock()
        self.temp_freq_hz       = None
        self.pressure_freq_hz   = None

        self.set_temp_pressure(temp_c, pressure_psi)

    def __str__(self):
        return 'FakeSensor(%s)' % self.serial_num

    def read_measurement(self):
        '''
        Returns a Measurement taken at the current crystal frequencies,
        including the simulated errors.  The perturbed pressure and temperature
        are converted back to crystal frequencies with compute_freqs(), and the
        Measurement's pressure and temperature are then evaluated from those
        frequencies, so the two match exactly, as they do on a real sensor.
        Raises ValueError if compute_freqs() finds no solution.
        '''
        with self.lock:
            fp = self.pressure_freq_hz
            ft = self.temp_freq_hz
        p  = self.p_poly(fp, ft)
        t  = self.t_poly(ft)
        p += self.p_error_offset_psi + self.p_error_slope * (p - AMBIENT_PSI)
        t += self.t_error_offset_c + self.t_error_slope * (t - AMBIENT_C)
        ft, fp = self.compute_freqs(t, p)
        return Measurement(self, self.time_ns_increasing(),
                           self.p_poly(fp, ft), self.t_poly(ft), fp, ft)

    def set_temp_pressure(self, temp_c, pressure_psi):
        '''
        Sets the crystal frequencies so that the sensor reads temp_c and
        pressure_psi, before any simulated errors.  Both frequencies change
        together, so a measurement read from another thread never pairs a
        new frequency with an old one.  Raises ValueError, leaving the
        frequencies unchanged, if compute_freqs() finds no solution.
        '''
        ft, fp = self.compute_freqs(temp_c, pressure_psi)
        with self.lock:
            self.temp_freq_hz     = ft
            self.pressure_freq_hz = fp

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
