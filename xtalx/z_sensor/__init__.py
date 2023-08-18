# Copyright (c) 2022-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import xtalx.tools.usb

from .tcsc_u5 import TCSC_U5
from .peak_tracker import PeakTracker
from .predicate_queue import PredicateQueue
from .sweeper import Sweeper


def find(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                product='XtalX TCSC', find_all=True, **kwargs)


def find_one(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    product='XtalX TCSC', find_all=True,
                                    **kwargs)


def make(usb_dev, **kwargs):
    if usb_dev.product == 'XtalX TCSC':
        return TCSC_U5(usb_dev, **kwargs)

    raise Exception('Unrecognized product string: %s' % usb_dev.product)


__all__ = ['find',
           'find_one',
           'make',
           'PeakTracker',
           'PredicateQueue',
           'Sweeper',
           ]
