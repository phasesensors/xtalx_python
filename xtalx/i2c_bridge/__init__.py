# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import xtalx.tools.usb

from .xi2c import XI2C


def find(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                product='XtalX XI2C', find_all=True, **kwargs)


def find_one(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    product='XtalX XI2C', find_all=True,
                                    **kwargs)


def make(usb_dev, **kwargs):
    return XI2C(usb_dev, **kwargs)


__all__ = ['find',
           'find_one',
           'make',
           ]
