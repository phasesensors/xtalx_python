# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import time
import math
import enum

import numpy as np

from xtalx.tools.math import Lorentzian


CHIRP_A      = 0.25
CHIRP_MS     = 105
CHIRP_DT     = CHIRP_MS * 0.001 * 2 + 0.02
CHIRP_MIN_RR = 0.2


class State(enum.Enum):
    # Waiting for some command.
    IDLE = 0

    # We have started a chirp to try and estimate the search parameters.
    CHIRP_WAIT_DATA = 1

    # We have started a low-resolution sweep to try and estimate the center
    # frequency and width.
    PEAK_SEARCH_WAIT_DATA = 2

    # We have started a hi-resolution sweep to get data for computing the
    # density and viscosity.
    HIRES_SWEEP_WAIT_DATA = 3


class Delegate:
    def chirp_callback(self, tc, pt, n, lf, X_data, Y_data, X_lorentzian,
                       Y_lorentzian):
        pass

    def sweep_started_callback(self, tc, pt, duration_ms, hires, f0, f1):
        pass

    def sweep_callback(self, tc, pt, t0_ns, duration_ms, points, fw_fit, hires,
                       temp_freq, temp_c):
        pass


class PeakTracker:
    def __init__(self, tc, amplitude, nfreqs, search_time, sweep_time,
                 settle_ms=2, delegate=Delegate(), enable_chirp=True):
        self.tc             = tc
        self.amplitude      = amplitude
        self.nfreqs         = nfreqs
        self.search_time    = search_time
        self.sweep_time     = sweep_time
        self.settle_ms      = settle_ms
        self.delegate       = delegate
        self.enable_chirp   = enable_chirp
        self.thread         = None
        self.thread_cond    = threading.Condition()

        self.start_time_ns  = None
        self.t_timeout      = None
        self.hires_f_center = None
        self.hires_width    = None
        self.min_rr         = None
        self.min_w          = None
        self.sensor_ms      = None
        self.sweep          = 0
        self.sweep_iter     = None
        self.sweep_t0_ns    = None
        self.state          = State.IDLE

        ci                = tc.crystal_info
        self.theta_deg    = ci.phase_shift_deg
        self.search_df    = ci.search_df
        self.search_min_f = min(ci.search_f0, ci.search_f1)
        self.search_max_f = max(ci.search_f0, ci.search_f1)
        self.chirp_space  = np.linspace(ci.chirp_f0, ci.chirp_f1,
                                        int(ci.chirp_f1 - ci.chirp_f0))

    def _transition(self, new_state):
        if self.state == new_state:
            return

        self.tc.dbg('Transition: %s -> %s' % (self.state, new_state))
        self.state = new_state

    def _start_sweep(self, ftups, hires):
        self.sweep_t0_ns   = time.time_ns()
        dt, self.sensor_ms = self.tc.sweep_async(self.amplitude, ftups,
                                                 ndiscards=self.settle_ms)
        self.t_timeout = time.time() + dt
        self.delegate.sweep_started_callback(self.tc, self, self.sensor_ms,
                                             hires, ftups[0][0], ftups[-1][0])

    def _read_sweep_points(self):
        points = self.tc.read_sweep_data(theta_deg=self.theta_deg).results
        max_amplitude = max(p.amplitude[1] for p in points)
        if max_amplitude >= self.tc.ADC_MAX / 2:
            self.tc.warn('Possible amplitude clipping.  Max amplitude '
                         'of %s exceeds maximum of %u.'
                         % (max_amplitude, self.tc.ADC_MAX // 2))

        return points

    def _get_sweep_fit(self, temp_hz):
        fit = self.tc.get_sweep_fit(temp_hz, theta_deg=self.theta_deg)
        temp_c = fit.temp_c
        if fit.status == 5:
            self.tc.warn('FwFit hit maximum iteration count, fit may be '
                         'sub-optimal.')
        elif not 1 <= fit.status <= 4:
            self.tc.warn('FwFit failed: %s s status %d niter %d' %
                         (fit.dt / 1e9, fit.status, fit.niter))
            fit = None

        return fit, temp_c

    def _start_full_search(self):
        if self.enable_chirp:
            self._start_chirp()
        else:
            self._start_peak_search_defaults()

    def _start_chirp(self):
        '''
        Start chirping and refine the data as it comes in.
        '''
        self.tc.info('Chirping from %f to %f...' % (self.chirp_space[0],
                                                    self.chirp_space[-1]))
        A = self.tc.a_to_dac(CHIRP_A)
        if A is None:
            A = 945
        self.tc.send_auto_chirp_cmd(self.chirp_space[0], self.chirp_space[-1],
                                    round(A))
        self._transition(State.CHIRP_WAIT_DATA)
        self.t_timeout = time.time() + CHIRP_DT
        self.sweep_iter = 0

    def _start_peak_search(self, min_f, max_f, df):
        '''
        Do a fast sweep of the full frequency range to try and find the peak.
        '''
        min_f       = math.floor(min_f / df) * df
        max_f       = math.ceil(max_f / df) * df
        N           = int(1 + (max_f - min_f) // df)
        dt          = math.ceil(1000 * self.search_time / N)
        freqs       = [min_f + i*df for i in range(N)]
        ftups       = [(f, dt) for f in freqs
                       if self.tc.crystal_info.is_valid_freq(f)]
        self.min_rr = 0.5
        self.min_w  = 12
        self._start_sweep(ftups, False)
        self._transition(State.PEAK_SEARCH_WAIT_DATA)

    def _start_peak_search_defaults(self):
        self._start_peak_search(self.search_min_f, self.search_max_f,
                                self.search_df)

    def _start_hires_sweep(self):
        '''
        Do a high-resolution sweep centered on the last peak we found.
        '''
        freqs       = self.tc.gen_hires_freqs(self.hires_f_center,
                                              self.hires_width,
                                              self.nfreqs // 2)
        dt          = math.ceil(1000 * self.sweep_time / len(freqs))
        ftups       = [(f, dt) for f in freqs
                       if self.tc.crystal_info.is_valid_freq(f)]
        self.min_rr = 0.75
        self.min_w  = 0
        self._start_sweep(ftups, True)
        self._transition(State.HIRES_SWEEP_WAIT_DATA)

    def _get_temp_freq(self):
        self.tc.set_t_enable(True)
        time.sleep(0.5)
        t0_crystal_ticks, t0_cpu_ticks = self.tc.read_temp()
        time.sleep(0.5)
        t1_crystal_ticks, t1_cpu_ticks = self.tc.read_temp()
        self.tc.set_t_enable(False)
        time.sleep(0.5)

        dt = (t1_cpu_ticks - t0_cpu_ticks) / self.tc.CPU_FREQ
        if dt == 0:
            return None

        dcrystal = (t1_crystal_ticks - t0_crystal_ticks) & 0xFFFFFFFF
        return dcrystal * 8 / dt

    def _handle_timeout(self):
        if self.state in (State.PEAK_SEARCH_WAIT_DATA,
                          State.HIRES_SWEEP_WAIT_DATA):
            self._handle_sweep_timeout()
        elif self.state == State.CHIRP_WAIT_DATA:
            self._handle_chirp_timeout()

    def _handle_sweep_timeout(self):
        points = self._read_sweep_points()
        points = points[1:]

        t0_ns = time.time_ns()

        temp_freq      = self._get_temp_freq() or 0
        fw_fit, temp_c = self._get_sweep_fit(temp_freq)
        hires          = (self.state == State.HIRES_SWEEP_WAIT_DATA)

        t1_ns = time.time_ns()
        dt_ms = round((t1_ns - t0_ns) / 1000000)

        if (hires and fw_fit is not None and fw_fit.RR >= self.min_rr and
                self.tc.crystal_info.is_valid_peak(fw_fit.peak_hz)):
            self.sweep_iter += 1
        else:
            self.sweep_iter = 0

        self.delegate.sweep_callback(self.tc, self, self.sweep_t0_ns,
                                     self.sensor_ms + dt_ms, points, fw_fit,
                                     hires, temp_freq, temp_c)
        self.sweep += 1

        if self.state == State.IDLE:
            return

        if fw_fit is None:
            self.tc.warn('Fit failed due to possible transient, repeating '
                         'peak search.')
            self._start_full_search()
            return

        if fw_fit.RR < self.min_rr:
            self.tc.warn('Confidence too low, repeating peak search.')
            self._start_full_search()
            return

        if not self.tc.crystal_info.is_valid_peak(fw_fit.peak_hz):
            self.tc.warn('Detected out-of-bounds peak, repeating peak search.')
            self._start_full_search()
            return

        self.hires_width    = max(abs(fw_fit.peak_fwhm), self.min_w)
        self.hires_f_center = fw_fit.peak_hz
        self._start_hires_sweep()

    def _handle_chirp_timeout(self):
        res = self.tc.sample_auto_chirp_sync()
        if res.bin1 >= len(res.bins):
            self.tc.info('Chirp out of range, falling back to search mode.')
            self._start_peak_search_defaults()
            return

        self.t_timeout = time.time() + CHIRP_DT

        X = np.linspace(res.f0, res.f1, res.nbins)
        Y = res.bins[res.bin0:res.bin1 + 1]
        try:
            lf = Lorentzian.from_x_y(X, Y, res.A, res.x0, res.W)
            Y2 = lf(self.chirp_space)
        except RuntimeError:
            lf = None
            Y2 = np.zeros(len(self.chirp_space))

        self.delegate.chirp_callback(self.tc, self, res.nchirps, lf, X, Y,
                                     self.chirp_space, Y2)

        if res.nchirps < 20:
            return

        self.tc._synchronize()

        if (lf and lf.RR >= CHIRP_MIN_RR and
                self.chirp_space[0] <= lf.x0 <= self.chirp_space[-1]):
            strength = lf.A / (math.pi * lf.W)
            self.tc.info('Chirp succeeded: peak_hz %s peak_fwhm %s '
                         'strength %.2f RR %.5f' %
                         (lf.x0, lf.W*2, strength, lf.RR))
            self.hires_width    = max(abs(lf.W * 2), 12)
            self.hires_f_center = lf.x0
            self._start_hires_sweep()
            return

        if lf:
            self.tc.info('Chirp failed with RR = %.5f' % lf.RR)
        else:
            self.tc.info('Chirp failed with no fit.')
        self._start_peak_search_defaults()

    def start_async(self):
        assert self.state == State.IDLE
        self.start_time_ns = time.time_ns()
        self._start_full_search()

    def stop_async(self):
        assert self.state != State.IDLE
        self.t_timeout = None
        self._transition(State.IDLE)

    def poll(self):
        # Process any pending timeout.
        if self.t_timeout is not None:
            t = time.time()
            if t >= self.t_timeout:
                self._handle_timeout()

        # If a timeout is armed, return the time delta for when next to poll.
        if self.t_timeout is not None:
            return max(self.t_timeout - time.time(), 0)

        # Poll continuously.
        return 0

    def start_threaded(self):
        with self.thread_cond:
            assert self.thread is None
            self.thread = threading.Thread(target=self._poll_threaded)
            self.thread.start()

    def stop_threaded(self):
        with self.thread_cond:
            t, self.thread = self.thread, None
            self.thread_cond.notify()
        t.join()

    def _poll_threaded(self):
        with self.thread_cond:
            if self.thread:
                self.start_time_ns = time.time_ns()
                self._start_full_search()
                while self.thread:
                    dt = self.poll()
                    self.thread_cond.wait(timeout=dt)
