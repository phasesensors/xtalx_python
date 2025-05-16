# Copyright (c) 2022-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import usb.core
import usb.backend.libusb1
import libusb_package


LIBUSB_BACKEND = None


def get_backend():
    global LIBUSB_BACKEND
    if not LIBUSB_BACKEND:
        LIBUSB_BACKEND = usb.backend.libusb1.get_backend(
                find_library=libusb_package.find_library)
    return LIBUSB_BACKEND


def find(serial_number=None, **kwargs):
    opt_args = {}
    if serial_number is not None:
        opt_args['serial_number'] = serial_number
    return list(usb.core.find(**kwargs, **opt_args, backend=get_backend()))


def find_one(**kwargs):
    usb_devs = find(**kwargs)
    if len(usb_devs) > 1:
        raise Exception('Multiple matching devices: %s' %
                        ', '.join(ud.serial_number for ud in usb_devs))
    if not usb_devs:
        return None
    return usb_devs[0]


__all__ = ['find',
           'get_backend',
           ]
