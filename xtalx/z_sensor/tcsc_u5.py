# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
from .tcsc import TCSC


class TCSC_U5(TCSC):
    DAC_MAX  = 4096
    ADC_MAX  = 16384
    ADC_KEYS = ('PROBEA', 'SIGIN')
