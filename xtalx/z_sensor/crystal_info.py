# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.


class CrystalInfo:
    def __init__(self, nominal_hz, chirp_f0, chirp_f1, search_f0, search_f1,
                 search_df, is_valid_freq_f0, is_valid_freq_f1,
                 is_valid_peak_f0, is_valid_peak_f1, phase_shift_deg=0):
        self.nominal_hz        = nominal_hz
        self.chirp_f0          = chirp_f0
        self.chirp_f1          = chirp_f1
        self.search_f0         = search_f0
        self.search_f1         = search_f1
        self.search_min_f      = min(search_f0, search_f1)
        self.search_max_f      = max(search_f0, search_f1)
        self.search_df         = search_df
        self.is_valid_freq_f0  = is_valid_freq_f0
        self.is_valid_freq_f1  = is_valid_freq_f1
        self.is_valid_peak_f0  = is_valid_peak_f0
        self.is_valid_peak_f1  = is_valid_peak_f1
        self.phase_shift_deg   = phase_shift_deg

    def is_valid_freq(self, hz):
        return self.is_valid_freq_f0 <= hz <= self.is_valid_freq_f1

    def is_valid_peak(self, hz):
        return self.is_valid_peak_f0 <= hz <= self.is_valid_peak_f1


CRYSTAL_INFOS = {
    # Gas crystal which operates at 20 kHz and only is used to measure density
    # and viscosity of gas.  This can have a very high Q-factor so when
    # searching the frequency delta should be small.
    20000 : CrystalInfo(20000, 19000, 21000, 19000, 20500, 10, 10000, 21000,
                        15000, 20000),

    # Fluid crystal which operates at 32.768 kHz and is used to measure air for
    # baseline purposes and otherwise fluids.  In fluid, this will have a very
    # low Q-factor so when searching the frequency delta can be very large.
    32768 : CrystalInfo(32768, 28000, 33500, 20000, 35000, 50, 10000, 45000,
                        15000, 35000),

    # Pressure crystal.  This is in vacuum and typically has a very high Q-
    # factor, so when searching the frequency delta should be small.
    50000 : CrystalInfo(50000, 48000, 52000, 48000, 52000, 15, 48000, 52000,
                        48000, 52000),

    # Temperature crystal.  This is in vacuum and has a very high Q-factor, so
    # when searching the frequency delta should be small.
    262000 : CrystalInfo(262000, 260000, 264000, 260000, 264000, 15,
                         260000, 264000, 260000, 264000, -155),
}
