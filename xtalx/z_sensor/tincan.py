# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time


class TinCan:
    def __init__(self, verbose=False, yield_Y=False):
        self.verbose         = verbose
        self.yield_Y         = yield_Y

    def log(self, tag, s, timestamp=None):
        timestamp = timestamp or time.time_ns()
        print('[%u] %s: %s' % (timestamp, tag, s))

    def info(self, s, **kwargs):
        self.log('I', s, **kwargs)

    def warn(self, s, **kwargs):
        self.log('W', s, **kwargs)

    def vlog(self, tag, s, **kwargs):
        if self.verbose:
            self.log(tag, s, **kwargs)

    def dbg(self, s, **kwargs):
        self.vlog('D', s, **kwargs)
