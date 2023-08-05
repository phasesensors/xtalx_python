# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .xti import XTI


def find_xti(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                product='XtalX', find_all=True, **kwargs)


def find_one_xti(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    product='XtalX', find_all=True, **kwargs)


def make(usb_dev, **kwargs):
    if usb_dev.product == 'XtalX':
        return XTI(usb_dev, **kwargs)

    raise Exception('Unrecognized product string: %s' % usb_dev.product)


__all__ = ['find_xti',
           'find_one_xti',
           'make',
           ]
