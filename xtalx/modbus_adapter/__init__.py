# Copyright (c) 2025 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .modbus_adapter import MBA, CommandException


def find_mba(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                bDeviceClass=0xFF, bDeviceSubClass=0x0C,
                                find_all=True, **kwargs)


def find_one_mba(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    bDeviceClass=0xFF, bDeviceSubClass=0x0C,
                                    find_all=True, **kwargs)


def make_mba(usb_dev, **kwargs):
    if usb_dev.bDeviceClass == 0xFF and usb_dev.bDeviceSubClass == 0x0C:
        return MBA(usb_dev, **kwargs)

    raise Exception('Unrecognized device: %s' % usb_dev)


__all__ = ['CommandException',
           'find_mba',
           'find_one_mba',
           'make_mba',
           'MBA',
           ]
