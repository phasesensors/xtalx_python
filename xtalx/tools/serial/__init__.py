# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import serial


def from_intf(intf, spy=False, **kwargs):
    url = ('spy://%s' % intf) if spy else intf
    return serial.serial_for_url(url, **kwargs)
