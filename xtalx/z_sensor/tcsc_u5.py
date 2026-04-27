# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
from . import tcsc_1xx
from . import tcsc_2xx


class TCSC_U5_1xx(tcsc_1xx.TCSC):
    DAC_MAX  = 4096
    ADC_MAX  = 16384
    ADC_KEYS = ('PROBEA', 'SIGIN')


class TCSC_U5_2xx(tcsc_2xx.TCSC):
    DAC_MAX  = 4096
    ADC_MAX  = 16384
    ADC_KEYS = ('PROBEA', 'SIGIN')
