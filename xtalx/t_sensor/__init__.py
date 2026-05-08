# Copyright (c) 2026 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .xtp import XTP


def find_one(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    bDeviceClass=0xFF, bDeviceSubClass=0x12,
                                    find_all=True, **kwargs)


def make(usb_dev, **kwargs):
    return XTP(usb_dev, **kwargs)


__all__ = [
    'find_one',
    'make',
    'XTP',
]
