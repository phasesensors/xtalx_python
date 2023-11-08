# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
from enum import IntEnum
import random
import usb.core
import usb.util

import btype

from . import i2c_bus


class Status(IntEnum):
    OK              = 0
    BAD_OPCODE      = 1
    BAD_LENGTH      = 2
    BAD_CMD_LENGTH  = 3
    BAD_BUS         = 4
    BAD_ADDR7       = 5
    FAILED          = 6
    DATA_OVERRUN    = 7
    DATA_UNDERRUN   = 8
    ABORTED         = 13

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            s = '%s' % Status(rsp.status)
        except ValueError:
            s = '%u' % rsp.status
        return s


class Opcode(IntEnum):
    BULK_READ   = 0x01
    BULK_WRITE  = 0x02
    ENABLE_BUS  = 0x03
    DISABLE_BUS = 0x04
    BAD_OPCODE  = 0xCCCC


class Command(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    params          = btype.Array(btype.uint32_t(), 7)
    _EXPECTED_SIZE  = 32


class Response(btype.Struct):
    opcode         = btype.uint16_t()
    tag            = btype.uint16_t()
    status         = btype.uint32_t()
    params         = btype.Array(btype.uint32_t(), 6)
    _EXPECTED_SIZE = 32


class BusFlags:
    ENABLE_PULLUP       = (1 << 0)


class XI2CCommandException(Exception):
    def __init__(self, rsp, rx_data):
        super().__init__(
            'Command exception: %s (%s)' % (Status.rsp_to_status_str(rsp), rsp))
        self.rsp = rsp
        self.rx_data = rx_data


class XI2CBus(i2c_bus.Bus):
    def __init__(self, xi2c, bus):
        self.xi2c = xi2c
        self.bus  = bus

    def enable(self, freq):
        self.xi2c.enable_bus(self.bus, freq)

    def enable_with_pullups(self, freq):
        self.xi2c.enable_bus_with_pullups(self.bus, freq)

    def disable(self):
        self.xi2c.disable_bus(self.bus)

    def read(self, addr7, nbytes):
        return self.xi2c.read_i2c(self.bus, addr7, nbytes)

    def write(self, addr7, data):
        self.xi2c.write_i2c(self.bus, addr7, data)


class XI2C:
    RSP_EP  = 0x81
    CMD_EP  = 0x02

    def __init__(self, usb_dev):
        self.usb_dev     = usb_dev

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 6)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise Exception(
                    'Device has no langid, ensure running as root!') from e

        self.tag = random.randint(1, 0xFFFF)

        self._set_configuration(usb_dev, 0x60)
        self.njunk_bytes = self._synchronize()

        self.busses = [
            XI2CBus(self, 1),
            XI2CBus(self, 2),
            XI2CBus(self, 3),
            XI2CBus(self, 4),
        ]

    @staticmethod
    def _set_configuration(usb_dev, bConfigurationValue, force=False):
        cfg = None
        try:
            cfg = usb_dev.get_active_configuration()
        except usb.core.USBError as e:
            if e.strerror != 'Configuration not set':
                raise

        if (cfg is None or cfg.bConfigurationValue != bConfigurationValue or
                force):
            usb.util.dispose_resources(usb_dev)
            usb_dev.set_configuration(bConfigurationValue)

    def _synchronize(self):
        '''
        Recover the connection if it was interrupted previously.  There could
        be a bunch of data posted on the RSP EP from a previous lifetime, so
        we post illegal commands to the probe until we get a 32-byte response.
        However, that 32-byte response could be the end of the previous life's
        transfer, so do a final illegal command and then check for the expected
        opcode and tag.
        '''
        self._send_abort()
        tag  = self._alloc_tag()
        cmd  = Command(opcode=Opcode.BAD_OPCODE, tag=tag)
        data = cmd.pack()
        junk = 0
        while True:
            self.usb_dev.write(self.CMD_EP, data)
            rsp = self.usb_dev.read(self.RSP_EP, 64)
            if len(rsp) == 32:
                break

            junk += len(rsp)

        self.usb_dev.write(self.CMD_EP, data)
        data = self.usb_dev.read(self.RSP_EP, 64)
        assert len(data) == 32

        rsp = Response.unpack(data)
        assert rsp.tag    == tag
        assert rsp.opcode == Opcode.BAD_OPCODE
        assert rsp.status == Status.BAD_OPCODE

        return junk

    def _send_abort(self):
        self.usb_dev.write(self.CMD_EP, b'')

    def _alloc_tag(self):
        tag      = self.tag
        self.tag = (self.tag + 1) & 0xFFFF
        return tag

    def _exec_command(self, opcode, params=None, bulk_data=b'', timeout=1000,
                      rx_len=0):
        if not params:
            params = [0, 0, 0, 0, 0, 0, 0]
        elif len(params) < 7:
            params = params + [0]*(7 - len(params))

        tag  = self._alloc_tag()
        cmd  = Command(opcode=opcode, tag=tag, params=params)
        data = cmd.pack()
        size = self.usb_dev.write(self.CMD_EP, data + bulk_data,
                                  timeout=timeout)
        assert size == len(data) + len(bulk_data)

        data = self.usb_dev.read(self.RSP_EP, Response._STRUCT.size + rx_len,
                                 timeout=timeout)
        assert len(data) >= Response._STRUCT.size

        rsp = Response.unpack(
                data[-Response._STRUCT.size:])          # pylint: disable=E1130
        rx_data = bytes(data[:-Response._STRUCT.size])  # pylint: disable=E1130
        assert rsp.tag == tag

        if rsp.status != Status.OK:
            rsp.opcode = Opcode(rsp.opcode)
            raise XI2CCommandException(rsp, rx_data)

        assert len(rx_data) == rx_len
        return rsp, rx_data

    def _enable_bus(self, bus, freq, flags):
        self._exec_command(Opcode.ENABLE_BUS, [bus, freq, flags])

    def enable_bus(self, bus, freq):
        self._enable_bus(bus, freq, 0)

    def enable_bus_with_pullups(self, bus, freq):
        self._enable_bus(bus, freq, BusFlags.ENABLE_PULLUP)

    def disable_bus(self, bus):
        self._exec_command(Opcode.DISABLE_BUS, [bus])

    def read_i2c(self, bus, addr7, nbytes):
        _, data = self._exec_command(Opcode.BULK_READ, [bus, addr7, nbytes],
                                     rx_len=nbytes)
        return data

    def write_i2c(self, bus, addr7, data):
        self._exec_command(Opcode.BULK_WRITE, [bus, addr7, len(data)],
                           bulk_data=data)
