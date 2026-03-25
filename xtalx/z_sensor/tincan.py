# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time


class TinCan:
    def __init__(self, verbose=False, yield_Y=False):
        self.verbose         = verbose
        self.yield_Y         = yield_Y
        self.serial_num      = None

    def log(self, tag, s, timestamp=None):
        timestamp = timestamp or time.time_ns()
        print('[%u - %s] %s: %s' % (timestamp, self.serial_num, tag, s))

    def info(self, s, **kwargs):
        self.log('I', s, **kwargs)

    def warn(self, s, **kwargs):
        self.log('W', s, **kwargs)

    def vlog(self, tag, s, **kwargs):
        if self.verbose:
            self.log(tag, s, **kwargs)

    def dbg(self, s, **kwargs):
        self.vlog('D', s, **kwargs)

    def make_influx_point(self, t0_ns, duration_ms, fw_fit, hires, temp_freq):
        p = {
            'measurement' : 'tincan_data',
            'time'        : t0_ns,
            'tags'        : {'sensor' : self.serial_num},
            'fields'      :
            {
                'hires'       : hires,
                'duration_ms' : duration_ms,
            }
        }

        if temp_freq is not None:
            p['fields']['temp_freq_hz'] = float(temp_freq)

        if fw_fit is None:
            return p

        p['fields'].update({
            'peak_hz'   : float(fw_fit.peak_hz),
            'peak_fwhm' : float(fw_fit.peak_fwhm),
            'RR'        : float(fw_fit.RR),
        })

        if fw_fit.temp_c is not None:
            p['fields']['temp_c'] = float(fw_fit.temp_c)
        if fw_fit.density_g_per_ml is not None:
            p['fields']['density_g_per_ml'] = float(fw_fit.density_g_per_ml)
        if fw_fit.viscosity_cp is not None:
            p['fields']['viscosity_cp'] = float(fw_fit.viscosity_cp)

        return p

    def make_stsdb_point(self, t0_ns, duration_ms, fw_fit, hires, temp_freq):
        p = {
            'time_ns'       : t0_ns,
            'hires'         : hires,
            'duration_ms'   : duration_ms,
        }

        if temp_freq is not None:
            p['temp_freq_hz'] = temp_freq

        if fw_fit is None:
            return p

        p.update({
            'peak_hz'   : fw_fit.peak_hz,
            'peak_fwhm' : fw_fit.peak_fwhm,
            'RR'        : fw_fit.RR,
        })

        if fw_fit.temp_c is not None:
            p['temp_c'] = fw_fit.temp_c
        if fw_fit.density_g_per_ml is not None:
            p['density_g_per_ml'] = fw_fit.density_g_per_ml
        if fw_fit.viscosity_cp is not None:
            p['viscosity_cp'] = fw_fit.viscosity_cp

        return p

    def make_stsdb_sweep_points(self, t0_ns, points):
        '''
        Given a list of ParsedSweepResults, convert this into a sequence of
        simple_tsdb points, with each of the sweep points 1 ns after the
        preceding one.  This "splats" all the points right after the t0_ns
        value for a given sweep.  Since each frequency in a sweep is measured
        for much longer than 1 ns, there is no danger of collision.
        '''
        return [
            {
                'time_ns'   : t0_ns + i,
                'f_hz'      : p.f,
                'z_real'    : p.Z.real,
                'z_imag'    : p.Z.imag,
                'RR'        : p.RR[1],
            }
            for i, p in enumerate(points)
        ]
