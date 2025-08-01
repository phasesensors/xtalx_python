# Copyright (c) 2025 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .spi_adapter import SPIA, CommandException


def find_spia(**kwargs):
    return xtalx.tools.usb.find(idVendor=0x0483, idProduct=0xA34E,
                                bDeviceClass=0xFF, bDeviceSubClass=0x0D,
                                find_all=True, **kwargs)


def find_one_spia(**kwargs):
    return xtalx.tools.usb.find_one(idVendor=0x0483, idProduct=0xA34E,
                                    bDeviceClass=0xFF, bDeviceSubClass=0x0D,
                                    find_all=True, **kwargs)


def make_spia(usb_dev, **kwargs):
    if usb_dev.bDeviceClass == 0xFF and usb_dev.bDeviceSubClass == 0x0D:
        return SPIA(usb_dev, **kwargs)

    raise Exception('Unrecognized device: %s' % usb_dev)


__all__ = ['CommandException',
           'find_spia',
           'find_one_spia',
           'make_spia',
           'SPIA',
           ]
