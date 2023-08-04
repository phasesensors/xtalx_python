# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import time
import math


def nop_callback(_sweep, _pos, _sd_results):
    pass


class SleepInterruptedException(Exception):
    pass


class Sweeper:
    def __init__(self, tc, amplitude, f0, f1, nfreqs, sweep_time,
                 min_freq_time, log10=True, settle_ms=2,
                 data_callback=nop_callback, nsweeps=0):
        self.sleep_cond     = threading.Condition()
        self.tc             = tc
        self.amplitude      = amplitude
        self.f0             = f0
        self.f1             = f1
        self.min_f          = min(f0, f1)
        self.max_f          = max(f0, f1)
        self.nfreqs         = nfreqs
        self.log10          = log10
        self.settle_ms      = settle_ms
        self.data_callback  = data_callback
        self.nsweeps        = nsweeps
        self.running        = False

        N = self.nfreqs
        if self.log10:
            k     = self.f1 / self.f0
            freqs = [self.f0 * k**(i/(N-1)) for i in range(N)]
        else:
            df    = (self.f1 - self.f0) / N
            freqs = [self.f0 + i*df for i in range(N)]

        fixed_time   = min_freq_time * N
        dynamic_time = sweep_time - fixed_time
        p = dynamic_time / sum(1 / f for f in freqs)
        self.freq_tuples = [(f, math.ceil(1000 * (min_freq_time + p / f)))
                            for f in freqs]

    def start(self):
        with self.sleep_cond:
            self.running = True
            threading.Thread(target=self.usb_thread).start()

    def stop(self):
        with self.sleep_cond:
            self.running = False
            self.sleep_cond.notify()

    def sleep_interruptible(self, dt, verbose=True):
        if verbose:
            wakeup_time = time.localtime(time.time() + dt)
            self.tc.info('Sleeping for %.3f seconds until %s...'
                         % (dt, time.asctime(wakeup_time)))

        with self.sleep_cond:
            start_time = time.time()
            while True:
                now     = time.time()
                elapsed = now - start_time
                if elapsed >= dt:
                    break
                if not self.running:
                    raise SleepInterruptedException()

                rem = dt - elapsed
                self.sleep_cond.wait(timeout=rem)

    def usb_thread(self):
        self.tc.info('Sweeping %u frequencies from %s Hz to %s Hz...'
                     % (self.nfreqs, self.f0, self.f1))

        pos   = 0
        sweep = 1
        while self.running:
            block  = self.freq_tuples[pos:pos+self.tc.ginfo.max_sweep_entries]
            nfreqs = len(block)
            t0_ns  = time.time_ns()
            dt, _  = self.tc.sweep_async(self.amplitude, block,
                                         ndiscards=self.settle_ms)
            try:
                self.sleep_interruptible(dt)
            except SleepInterruptedException:
                break
            sd = self.tc.read_sweep_data()

            for r, b in zip(sd.results, block):
                assert r.nbufs == b[1]

            self.data_callback(sweep, t0_ns, pos, sd.results, self.amplitude)

            if pos + nfreqs == len(self.freq_tuples):
                self.tc.info('Sweep %u complete.' % sweep)
                pos = 0
                if sweep == self.nsweeps:
                    break
                sweep += 1
            else:
                pos += nfreqs

        self.tc.info('Sweeper stopped.')
        self.running = False
