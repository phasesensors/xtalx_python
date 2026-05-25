# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .xhti import XHTI
from .xhtis import XHTIS
from .xhtism import XHTISM
from .xhtiss import XHTISS
from .xmhti import XMHTI
from .xti import XTI
from .xti15 import XTI15


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


def find_xti15(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                bDeviceClass=0xFF, bDeviceSubClass=0x13,
                                find_all=True, **kwargs)


def find_one_xti15(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    bDeviceClass=0xFF, bDeviceSubClass=0x13,
                                    find_all=True, **kwargs)


def make_xti15(usb_dev, **kwargs):
    if usb_dev.bDeviceClass == 0xFF and usb_dev.bDeviceSubClass == 0x13:
        return XTI15(usb_dev, **kwargs)

    raise Exception('Unrecognized device: %s' % usb_dev)


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
           'XTI15',
           'XHTI',
           'XHTIS',
           'XHTISM',
           'XHTISS',
           'XMHTI',
           ]
