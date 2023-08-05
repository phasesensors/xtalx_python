# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.

class SignalInfo:
    def __init__(self, X, Y, clk, name, phasor=None, offset=None, RR=None):
        self.phasor  = phasor
        self.offset  = offset
        self.RR      = RR
        self.X       = X
        self.Y       = Y
        self.clk     = clk
        self.name    = name


class ScopeData:
    def __init__(self, freq, w, hdr=None, data=None):
        self.freq     = freq
        self.w        = w
        self.hdr      = hdr
        self.data     = data
        self.sig_info = []
        self.map      = {}

    def __getitem__(self, key):
        return self.map[key]

    def add_signal(self, X, Y, clk, name, **kwargs):
        self.sig_info.append(SignalInfo(X, Y, clk, name, **kwargs))
        if name:
            self.map[name] = self.sig_info[-1]
