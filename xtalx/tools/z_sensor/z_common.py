# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import math

import xtalx.z_sensor
from xtalx.tools.config import Config
from xtalx.tools.influxdb import InfluxDBPushQueue


class ZArgs:
    '''
    Helper class to hold common arguments parsed from the command line.
    '''
    def __init__(self, amplitude, nfreqs, search_time_secs, sweep_time_secs,
                 settle_ms):
        self.amplitude        = amplitude
        self.nfreqs           = nfreqs
        self.search_time_secs = search_time_secs
        self.sweep_time_secs  = sweep_time_secs
        self.settle_ms        = settle_ms


class ZLogger:
    '''
    Helper class that takes care of logging measurements to the console as well
    as writing fit and dump files, if specified.
    '''
    def __init__(self, fit_file, dump_file):
        if dump_file:
            dump_file = open(  # pylint: disable=R1732
                    dump_file, 'a', encoding='utf8')
            dump_file.write('sweep,time_ns,f,dt,zx_real,zx_imag,RR,amplitude\n')
            dump_file.flush()
        else:
            dump_file = None

        if fit_file:
            fit_file = open(  # pylint: disable=R1732
                    fit_file, 'a', encoding='utf8')
            fit_file.write('time_ns,sweep,hires,peak_hz,peak_fwhm,RR,temp_c,'
                           'density_g_per_ml,viscosity_cp,amplitude\n')
            fit_file.flush()
        else:
            fit_file = None

        self.fit_file  = fit_file
        self.dump_file = dump_file

    def log_chirp(self, tc, nchirps, lf):
        if lf is not None:
            tc.log('C', 'i %u peak_hz %.3f peak_fwhm %.3f RR %.6f' %
                   (nchirps - 1, lf.x0, 2 * lf.W, lf.RR))
        else:
            tc.log('c', 'i %u' % nchirps)

    def log_sweep(self, tc, pt, t0_ns, points, fw_fit, hires, temp_freq,
                  temp_c):
        tag = 'H' if hires else 'S'
        f0  = points[0].f
        f1  = points[-1].f
        if fw_fit is None:
            T, D, V = temp_c, None, None
            tc.log(tag, 'i %u f0 %f f1 %f temp_hz %s T %s D %s V %s' %
                   (pt.sweep, f0, f1, temp_freq, T, D, V))
        else:
            T = temp_c
            D = fw_fit.density_g_per_ml
            V = fw_fit.viscosity_cp
            tc.log(tag, 'i %u f0 %f f1 %f peak_hz %.3f peak_fwhm %.3f S %.2f '
                   'RR %.6f temp_hz %s T %s D %s V %s' %
                   (pt.sweep, f0, f1, fw_fit.peak_hz, fw_fit.peak_fwhm,
                    fw_fit.strength, fw_fit.RR, temp_freq, T, D, V))

        if self.dump_file:
            for p in points:
                self.dump_file.write('%s,%u,%s,%.3f,%s,%s,%s,%u\n' %
                                     (pt.sweep, t0_ns, p.f, p.nbufs / 1000.,
                                      p.z.real, p.z.imag, p.RR[1],
                                      pt.amplitude))
            self.dump_file.flush()

        if self.fit_file:
            self.fit_file.write(
                '%u,%s,%u,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%u\n' %
                (t0_ns, pt.sweep,
                 1 if hires else 0,
                 fw_fit.peak_hz if fw_fit else math.nan,
                 fw_fit.peak_fwhm if fw_fit else math.nan,
                 fw_fit.RR if fw_fit is not None else math.nan,
                 T if T is not None else math.nan,
                 D if D is not None else math.nan,
                 V if V is not None else math.nan,
                 pt.amplitude))
            self.fit_file.flush()

        return T, D, V


class ZDelegate(xtalx.z_sensor.peak_tracker.Delegate):
    '''
    Helper delegate class that simply logs sweeps using the ZLogger object.
    '''
    def __init__(self, z_logger):
        self.z_logger = z_logger

    def chirp_callback(self, tc, _pt, n, lf, _X_data, _Y_data, _X_lorentzian,
                       _Y_lorentzian):
        self.z_logger.log_chirp(tc, n, lf)

    def sweep_callback(self, tc, pt, t0_ns, _duration_ms, points, fw_fit,
                       hires, temp_freq, temp_c):
        self.z_logger.log_sweep(tc, pt, t0_ns, points, fw_fit, hires, temp_freq,
                                temp_c)


def parse_config(rv):
    if not rv.config:
        return None, None

    with open(rv.config, encoding='utf8') as f:
        c = Config(f.readlines(), ['influx_host', 'influx_user',
                                   'influx_password', 'influx_database'])

        ipq = InfluxDBPushQueue(c.influx_host, 8086, c.influx_user,
                                c.influx_password, database=c.influx_database,
                                ssl=True, verify_ssl=True, timeout=100)

    return c, ipq


def parse_args(tc, rv):
    if tc.yield_Y:
        tc.info('Tracking admittance.')
    else:
        tc.info('Tracking impedance.')

    amplitude = tc.parse_amplitude(rv.amplitude)

    tc.info('Using amplitude of %u DAC codes.' % amplitude)
    volts = tc.dac_to_a(amplitude)
    if volts is not None:
        tc.info('Using amplitude of %f V.' % volts)

    if not 0 <= amplitude <= 2000:
        raise Exception('Amplitude not in range 0 to 2000.')

    return (ZArgs(amplitude, rv.nfreqs, rv.search_time_secs, rv.sweep_time_secs,
                  rv.settle_ms),
            ZLogger(rv.fit_file, rv.dump_file))


def add_arguments(parser):
    parser.add_argument('--amplitude', '-a')
    parser.add_argument('--nfreqs', '-n', type=int, default=100)
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--search-time-secs', type=float, default=30)
    parser.add_argument('--sweep-time-secs', type=float, default=30)
    parser.add_argument('--settle-ms', type=int, default=5000)
    parser.add_argument('--dump-file', '-d')
    parser.add_argument('--fit-file', '-f')
    parser.add_argument('--sensor', '-s')
    parser.add_argument('--config', '-c')
    parser.add_argument('--track-impedance', action='store_true')
