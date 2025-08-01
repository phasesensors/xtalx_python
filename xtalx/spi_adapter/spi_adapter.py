# Copyright (c) 2025 Phase Advanced Sensor Systems, Inc.
from enum import IntEnum
import platform
import random

import usb
import usb.util
import btype


class Status(IntEnum):
    OK              = 0
    ABORTED         = 1
    BAD_OPCODE      = 2
    BAD_CMD_LENGTH  = 3
    BAD_DATA_LENGTH = 4
    BAD_ADDR        = 5
    BAD_PARAM       = 6
    XACT_FAILED     = 7

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            s = '%s' % Status(rsp.status)
        except ValueError:
            s = '%u' % rsp.status
        return s


class Opcode(IntEnum):
    SET_VEXT                 = 0x0001
    SET_FREQUENCY            = 0x0002
    XACT                     = 0x0003
    JUMP_BOOTLOADER          = 0x7B42


class Command(btype.Struct):
    opcode          = btype.uint16_t()
    tag             = btype.uint16_t()
    data_len        = btype.uint16_t()
    rsrv            = btype.uint16_t()
    params          = btype.Array(btype.uint32_t(), 6)
    _EXPECTED_SIZE  = 32


class Response(btype.Struct):
    opcode         = btype.uint16_t()
    tag            = btype.uint16_t()
    data_len       = btype.uint16_t()
    status         = btype.uint16_t()
    params         = btype.Array(btype.uint32_t(), 6)
    _EXPECTED_SIZE = 32


class CommandException(Exception):
    def __init__(self, rsp, rx_data):
        super().__init__(
            'Command exception: %s (%s)' % (Status.rsp_to_status_str(rsp), rsp))
        self.rsp = rsp
        self.rx_data = rx_data


class SPIA:
    CMD_EP = 0x01
    RSP_EP = 0x82

    def __init__(self, usb_dev):
        super().__init__()

        self.usb_dev      = usb_dev
        self.tag          = random.randint(0, 65535)
        self.last_time_ns = 0

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 6)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise Exception(
                    'Device has no langid, ensure running as root!') from e

        self.fw_version_str = '%u.%u.%u' % (
                (self.fw_version >> 8) & 0xFF,
                (self.fw_version >> 4) & 0x0F,
                (self.fw_version >> 0) & 0x0F)

        self._synchronize()

    def __str__(self):
        return 'SPIA(%s)' % self.serial_num

    def _exec_command(self, opcode, params=None, data=b'', timeout_ms=1000):
        if not params:
            params = [0, 0, 0, 0, 0, 0]
        elif len(params) < 6:
            params = params + [0]*(6 - len(params))

        tag = self.tag
        self.tag = (self.tag + 1) & 0xFFFF

        cmd = Command(opcode=opcode, tag=tag, data_len=len(data),
                      params=params)
        l = self.usb_dev.write(self.CMD_EP, cmd.pack(), timeout=timeout_ms)
        assert l == 32

        if data:
            l = self.usb_dev.write(self.CMD_EP, data, timeout=timeout_ms)
            assert l == len(data)

        data = self.usb_dev.read(self.RSP_EP, Response._STRUCT.size,
                                 timeout=timeout_ms)
        assert len(data) == Response._STRUCT.size
        rsp = Response.unpack(data)
        assert rsp.opcode == opcode
        assert rsp.tag    == tag

        if rsp.data_len:
            rx_data = bytes(self.usb_dev.read(self.RSP_EP, rsp.data_len,
                                              timeout=timeout_ms))
            assert len(rx_data) == rsp.data_len
        else:
            rx_data = None

        if rsp.status != Status.OK:
            raise CommandException(rsp, rx_data)

        return rsp, rx_data

    def _set_configuration(self, bConfigurationValue):
        cfg = None
        try:
            cfg = self.usb_dev.get_active_configuration()
        except usb.core.USBError as e:
            if e.strerror != 'Configuration not set':
                raise

        if cfg is None or cfg.bConfigurationValue != bConfigurationValue:
            usb.util.dispose_resources(self.usb_dev)
            self.usb_dev.set_configuration(bConfigurationValue)

    def _synchronize(self):
        self._set_configuration(0x84)

        # Try to abort any existing command.  Depending on the adapter's state:
        #   Waiting to RX CMD: will send a US_ABORTED RSP.
        #   Waiting to RX DATA: will send a US_ABORTED RSP.
        #   Trying to TX RSP: write will time out, read skipped over.
        #   Trying to TX DATA: write will time out, read skipped over.
        #
        # The best way to abort this would be to write a zero-length packet to
        # CMD_EP.  The write should time out and there is no danger of the
        # write completing a data phase.  However, on Windows a zero-length
        # write doesn't cause a timeout, it just silently succeeds even if the
        # target isn't accepting data on that endpoint.  Since we rely on the
        # timeout to detect what state we are in, that breaks the algorithm.
        # So, on Windows we do a 1-byte write instead, which hopefully is safe
        # in almost every case.
        try:
            if platform.system() == 'Windows':
                self.usb_dev.write(self.CMD_EP, b'\x00', timeout=100)
            else:
                self.usb_dev.write(self.CMD_EP, b'', timeout=100)
            data = self.usb_dev.read(self.RSP_EP, 32, timeout=100)
            assert len(data) == 32
            rsp = Response.unpack(data)
            if platform.system() == 'Windows':
                assert rsp.status in (Status.BAD_CMD_LENGTH,
                                      Status.BAD_DATA_LENGTH)
            else:
                assert rsp.status == Status.ABORTED
            return
        except usb.core.USBTimeoutError:
            pass

        # The adapter is either transmitting a RSP or a DATA sequence.  We can't
        # be sure which so we need to do two reads.  The first read will always
        # succeed and the second read will time out if there was no DATA phase
        # associated with the RSP.
        self.usb_dev.read(self.RSP_EP, 256, timeout=100)
        try:
            self.usb_dev.read(self.RSP_EP, 256, timeout=100)
        except usb.core.USBTimeoutError:
            pass

    def set_vext(self, enabled):
        return self._exec_command(Opcode.SET_VEXT, [enabled])

    def set_spi_frequency(self, hz):
        rsp, _ = self._exec_command(Opcode.SET_FREQUENCY, [hz])
        return rsp.params[0]

    def enter_dfu_mode(self):
        return self._exec_command(Opcode.JUMP_BOOTLOADER, [0xA47B39FE])

    def transact(self, data):
        rsp, rx_data = self._exec_command(Opcode.XACT, data=data)

        assert rsp.data_len == len(data)
        assert rsp.data_len == len(rx_data)

        return rx_data
