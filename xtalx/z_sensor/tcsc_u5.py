# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
from . import tcsc_1xx


class TCSC_U5_1xx(tcsc_1xx.TCSC):
    DAC_MAX  = 4096
    ADC_MAX  = 16384
    ADC_KEYS = ('PROBEA', 'SIGIN')
