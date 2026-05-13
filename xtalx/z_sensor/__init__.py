# Copyright (c) 2022-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import xtalx.tools.usb

from .tcsc_u5 import TCSC_U5
from .peak_tracker import PeakTracker
from .predicate_queue import PredicateQueue
from .sweeper import Sweeper
from . import tcsc_1xx
from . import tcsc_2xx


def find(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                bDeviceClass=0xFF, bDeviceSubClass=0x01,
                                find_all=True, **kwargs)


def find_one(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    bDeviceClass=0xFF, bDeviceSubClass=0x01,
                                    find_all=True, **kwargs)


def make(usb_dev, **kwargs):
    comms = None
    if usb_dev.bcdDevice >= 0x200:
        comms = tcsc_2xx.Comms(usb_dev)
    elif usb_dev.product == 'XtalX TCSC':
        comms = tcsc_1xx.Comms(usb_dev)
    else:
        raise Exception('Unrecognized product string: %s' % usb_dev.product)

    return TCSC_U5(comms, **kwargs)


__all__ = ['find',
           'find_one',
           'make',
           'PeakTracker',
           'PredicateQueue',
           'Sweeper',
           ]
