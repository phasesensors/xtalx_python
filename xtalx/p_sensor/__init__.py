# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import xtalx.tools.usb

from .xhti import XHTI
from .xhtism import XHTISM
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


def match_xmhti(d):
    # Match on Vendor and Product IDs first.
    if d.idVendor != 0x0483:
        return False
    if d.idProduct != 0xA34E:
        return False

    # Old-style XMHTI are explicit about class and sub-class.
    if d.bDeviceClass == 0xFF and d.bDeviceSubClass == 0x05:
        return True

    # New-style XMHTI adhere to iAP.
    if d.bDeviceClass != 0x00:
        return False
    if d.bDeviceSubClass != 0x00:
        return False
    if d.bDeviceProtocol != 0x00:
        return False

    # We should only have 1 configuration with 2 interfaces.
    cfgs = d.configurations()
    if len(cfgs) != 1:
        return False
    ifs = cfgs[0].interfaces()
    if len(ifs) != 2:
        return False

    # Validate interface 0.
    if0 = ifs[0]
    if if0.bInterfaceClass != 0xFF:
        return False
    if if0.bInterfaceSubClass != 0xFF:
        return False
    if if0.bInterfaceProtocol != 0x50:
        return False
    if len(if0.endpoints()) != 3:
        return False

    # Validate interface 1.
    if1 = ifs[1]
    if if1.bInterfaceClass != 0xFF:
        return False
    if if1.bInterfaceSubClass != 0xF0:
        return False
    if if1.bInterfaceProtocol != 0x00:
        return False
    if len(if1.endpoints()) != 2:
        return False

    return True


def find_xmhti(**kwargs):
    return xtalx.tools.usb.find(custom_match=match_xmhti, find_all=True,
                                **kwargs)


def find_one_xmhti(**kwargs):
    return xtalx.tools.usb.find_one(custom_match=match_xmhti, find_all=True,
                                    **kwargs)


def make_xmhti(usb_dev, **kwargs):
    if match_xmhti(usb_dev):
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
           'XMHTI',
           ]
