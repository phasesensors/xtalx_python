# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import time
import enum

from .peak_tracker import Delegate


class State(enum.Enum):
    # Discarding measurements except for the most recent.
    IDLE = 0

    # Queueing measurements until a condition is satisfied.
    QUEUEING = 1

    # Predicate has matched.
    MATCHED = 2

    # Matching aborted.
    ABORTED = 3


class Measurement:
    def __init__(self, t0_ns, duration_ms, points, fw_fit, temp_freq, temp_c):
        self.t0_ns            = t0_ns
        self.duration_ms      = duration_ms
        self.points           = points
        self.fw_fit           = fw_fit
        self.temp_freq        = temp_freq
        self.temp_c           = temp_c
        if fw_fit:
            self.peak_hz          = fw_fit.peak_hz
            self.peak_fwhm        = fw_fit.peak_fwhm
            self.density_g_per_ml = fw_fit.density_g_per_ml
            self.viscosity_cp     = fw_fit.viscosity_cp
        else:
            self.peak_hz          = None
            self.peak_fwhm        = None
            self.density_g_per_ml = None
            self.viscosity_cp     = None

    def __repr__(self):
        return ('Measurement(peak_hz=%f, peak_fwhm=%f)' %
                (self.peak_hz, self.peak_fwhm))


class PredicateQueue(Delegate):
    def __init__(self, delegate=None):
        self.queue_cond = threading.Condition()
        self.queue      = []
        self.state      = State.IDLE
        self.predicate  = None
        self.N          = None
        self.delegate   = delegate

    def chirp_callback(self, *args):
        if self.delegate:
            self.delegate.chirp_callback(*args)

    def sweep_callback(self, tc, pt, t0_ns, duration_ms, points, fw_fit, hires,
                       temp_freq, temp_c):
        if self.delegate:
            self.delegate.sweep_callback(tc, pt, t0_ns, duration_ms, points,
                                         fw_fit, hires, temp_freq, temp_c)

        m = Measurement(t0_ns, duration_ms, points, fw_fit, temp_freq, temp_c)

        with self.queue_cond:
            if not hires:
                self.queue.clear()
                return

            self.queue.append(m)

            if self.state in (State.IDLE, State.MATCHED):
                return

            if self.state == State.QUEUEING:
                if len(self.queue) >= self.N:
                    if self.predicate(
                            self.queue[-self.N:]):  # pylint: disable=E1130
                        self.state = State.MATCHED
                        self.queue_cond.notify()

    def _wait_predicate_locked(self, predicate, N, timeout):
        self.state     = State.QUEUEING
        self.predicate = predicate
        self.N         = N

        start_time = time.time()
        rem        = timeout
        while True:
            if self.state == State.MATCHED:
                return True
            if ((rem is not None and rem <= 0) or
                    self.state == State.ABORTED):
                self.state = State.IDLE
                return False

            if timeout:
                self.queue_cond.wait(timeout=rem)
                rem = start_time + timeout - time.time()
            else:
                self.queue_cond.wait()

    def wait_predicate(self, predicate, N, timeout=None):
        '''
        Wait until the specified predicate returns true.  The predicate is a
        callable which takes a single argument, measurements, a list of N
        measurements.  The callable should return True if the predicate
        condition is matched (for instance, the standard deviation of all the
        N measurements is below a critical value), or False otherwise.

        The queue is not cleared when wait_predicate() is invoked, however at
        least one measurement must come in before the call will return - i.e.
        if the predicate would currently match against the elements currently
        in the queue, it will still wait for at least one more measurement to
        be made before evaluating the predicate.

        When the predicate matches, a list of the N measurements used in the
        evaluation is returned.  If the predicate fails to match in the timeout
        period, None is returned.
        '''
        with self.queue_cond:
            if self._wait_predicate_locked(predicate, N, timeout):
                return self.queue[:]
        return None

    def get_measurements(self):
        '''
        Returns a list of all measurements made since the last time the queue
        was cleared.  The queue will clear automatically when a lo-res (search)
        measurement is made, so only usable measurements are returned.
        '''
        with self.queue_cond:
            return self.queue[:]

    def get_measurement(self):
        '''
        Returns the most recent measurement, or None if no measurement is
        available yet.
        '''
        with self.queue_cond:
            if not self.queue:
                return None
            return self.queue[0]

    def clear(self):
        '''
        Clears all measurements from the queue so that matching will only be
        performed on new measurements.
        '''
        with self.queue_cond:
            self.queue.clear()

    def abort(self):
        '''
        Abort any wait_predicate() call in progress.
        '''
        with self.queue_cond:
            self.state = State.ABORTED
            self.queue_cond.notify()
