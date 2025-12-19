# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .xhti import XHTI
from .xhtism import XHTISM
from .xhtiss import XHTISS
from .xmhti import XMHTI
from .xti import XTI


def find_xti(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                product='XtalX', find_all=True, **kwargs)


def find_one_xti(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    product='XtalX', find_all=True, **kwargs)


def make_xti(usb_dev, **kwargs):
    if usb_dev.product == 'XtalX':
        return XTI(usb_dev, **kwargs)

    raise Exception('Unrecognized product string: %s' % usb_dev.product)


def find_xmhti(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                bDeviceClass=0xFF, bDeviceSubClass=0x05,
                                find_all=True, **kwargs)


def find_one_xmhti(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    bDeviceClass=0xFF, bDeviceSubClass=0x05,
                                    find_all=True, **kwargs)


def make_xmhti(usb_dev, **kwargs):
    if usb_dev.bDeviceClass == 0xFF and usb_dev.bDeviceSubClass == 0x05:
        return XMHTI(usb_dev, **kwargs)

    raise Exception('Unrecognized device: %s' % usb_dev)


def make(usb_dev, **kwargs):
    if usb_dev.bDeviceClass == 0xFF and usb_dev.bDeviceSubClass == 0x05:
        return XMHTI(usb_dev, **kwargs)
    if usb_dev.product == 'XtalX':
        return XTI(usb_dev, **kwargs)

    raise Exception('Unrecognized product string: %s' % usb_dev.product)


__all__ = ['find_xti',
           'find_one_xti',
           'make_xti',
           'find_xmhti',
           'find_one_xmhti',
           'make_xmhti',
           'make',
           'XTI',
           'XHTI',
           'XHTISM',
           'XHTISS',
           'XMHTI',
           ]
