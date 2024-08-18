# Copyright (c) 2022 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import struct

import btype

from xtalx.tools.math import (PolynomialFit1D, PolynomialFit2D,
                              make_poly_bstruct)


CAL_F0_REFCLK           = (1 << 0)
CAL_F0_POLY_PSI         = (1 << 1)
CAL_F0_POLY_TEMP        = (1 << 2)
CAL_F0_COMP_STANDARD    = (1 << 3)
CAL_F0_REPORT_ID        = (1 << 4)
CAL_F0_OSC_STARTUP_MS   = (1 << 5)

CAL_FEATURES = [{CAL_F0_REFCLK         : 'MCU refclk',
                 CAL_F0_POLY_PSI       : 'PSI polynomial',
                 CAL_F0_POLY_TEMP      : 'Temperature polynomial',
                 CAL_F0_COMP_STANDARD  : 'Standard Comparators',
                 CAL_F0_REPORT_ID      : 'XtalxDB Calibration Report ID',
                 CAL_F0_OSC_STARTUP_MS : 'Oscillator startup period in ms',
                 },
                {},
                {},
                {},
                {},
                ]

CAL_SIG = 0x4C414358


def checksum_xor32(mem):
    assert (len(mem) % 4) == 0
    checksum = 0
    for offset in range(0, len(mem), 4):
        checksum ^= struct.unpack_from('<L', mem, offset)[0]
    return checksum


PolyPSI  = make_poly_bstruct(2, 25)
PolyTemp = make_poly_bstruct(1, 5)


class CalPage(btype.Struct):
    # Ensure this section is a multiple of 8 bytes.
    sig                 = btype.uint32_t(0xFFFFFFFF)
    checksum_xor32      = btype.uint32_t(0xFFFFFFFF)
    len                 = btype.uint32_t(0xFFFFFFFF)
    features            = btype.Array(btype.uint32_t(0xFFFFFFFF), 5)
    refclk_x10          = btype.uint32_t(0xFFFFFFFF)
    report_id           = btype.uint32_t(0xFFFFFFFF)
    poly_psi            = PolyPSI()
    poly_temp           = PolyTemp()
    osc_startup_time_ms = btype.uint32_t(0xFFFFFFFF)
    pad                 = btype.Array(btype.uint32_t(0xFFFFFFFF), 1)

    # Reserved area, should also be a multiple of 8 bytes.
    rsrv2               = btype.Array(btype.uint32_t(0xFFFFFFFF), 424)
    _EXPECTED_SIZE      = 2048

    @staticmethod
    def get_short_size():
        return CalPage._EXPECTED_SIZE - CalPage._TYPE_MAP['rsrv2']._N * 4

    def is_valid(self):
        if self.sig != CAL_SIG:
            return False
        if self.len > CalPage._EXPECTED_SIZE:
            return False

        self.poly_psi.pack()
        self.poly_temp.pack()

        data = self.pack()
        if checksum_xor32(data[:self.len]) != 0:
            return False

        return True

    def is_same(self, other):
        return self.pack() == other.pack()

    def get_refclk_x10(self):
        if self.features[0] & CAL_F0_REFCLK:
            return self.refclk_x10
        return None

    def get_report_id(self):
        if self.features[0] & CAL_F0_REPORT_ID:
            return self.report_id
        return None

    def get_osc_startup_time_ms(self):
        if self.features[0] & CAL_F0_OSC_STARTUP_MS:
            return self.osc_startup_time_ms
        return None

    def get_polynomials(self):
        if self.features[0] & CAL_F0_POLY_PSI:
            poly_psi = PolynomialFit2D.from_poly_bstruct(self.poly_psi)
        else:
            poly_psi = None

        if self.features[0] & CAL_F0_POLY_TEMP:
            poly_temp = PolynomialFit1D.from_poly_bstruct(self.poly_temp)
        else:
            poly_temp = None

        return poly_psi, poly_temp
