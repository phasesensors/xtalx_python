# Copyright (c) 2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time


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


class FakeSensor:
    '''
    Simulated pressure sensor for testing tools without hardware.

    serial_num is the sensor's serial number string.  t_poly and p_poly are
    calibration polynomials in the same form as those used by
    CalPage.poly_psi and CalPage.poly_temp: t_poly(Ft) gives the
    temperature in degrees C and p_poly(Fp, Ft) gives the pressure in PSI,
    where Fp and Ft are the pressure and temperature crystal frequencies in
    Hz.  The crystal frequencies are taken from the pressure_freq_hz and
    temp_freq_hz fields, which may be changed at any time to simulate a
    change in pressure or temperature.
    '''
    def __init__(self, serial_num, t_poly, p_poly, pressure_freq_hz=52000,
                 temp_freq_hz=262000):
        self.serial_num       = serial_num
        self.t_poly           = t_poly
        self.p_poly           = p_poly
        self.pressure_freq_hz = pressure_freq_hz
        self.temp_freq_hz     = temp_freq_hz
        self.last_time_ns     = 0

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

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
