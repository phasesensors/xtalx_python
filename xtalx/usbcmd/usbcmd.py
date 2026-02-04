# Copyright (c) 2025-2026 Phase Advanced Sensor Systems, Inc.
from enum import IntEnum
import platform
import random
import time

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

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            s = '%s' % Status(rsp.status)
        except ValueError:
            s = '%u' % rsp.status
        return s


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


class Device:
    def __init__(self, usb_dev, cmd_ep, rsp_ep, max_cmd_data_len,
                 max_rsp_data_len, git_sha1_index=None,
                 default_configuration=None):
        super().__init__()

        self.usb_dev          = usb_dev
        self.cmd_ep           = cmd_ep
        self.rsp_ep           = rsp_ep
        self.max_cmd_data_len = max_cmd_data_len
        self.max_rsp_data_len = max_rsp_data_len
        self.tag              = random.randint(0, 65535)
        self.last_time_ns     = 0

        try:
            self.serial_num = usb_dev.serial_number
            if git_sha1_index is not None:
                self.git_sha1 = usb.util.get_string(usb_dev, git_sha1_index)
            else:
                self.git_sha1 = None
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise Exception(
                    'Device has no langid, ensure running as root!') from e

        self.fw_version_str = '%u.%u.%u' % (
                (self.fw_version >> 8) & 0xFF,
                (self.fw_version >> 4) & 0x0F,
                (self.fw_version >> 0) & 0x0F)

        if default_configuration is not None:
            self._set_configuration(default_configuration)
        self._synchronize()

    def _exec_command(self, opcode, params=None, data=b'', timeout_ms=1000):
        if data:
            assert len(data) <= self.max_cmd_data_len
        if not params:
            params = [0, 0, 0, 0, 0, 0]
        elif len(params) < 6:
            params = params + [0]*(6 - len(params))

        tag = self.tag
        self.tag = (self.tag + 1) & 0xFFFF

        cmd = Command(opcode=opcode, tag=tag, data_len=len(data),
                      params=params)
        l = self.usb_dev.write(self.cmd_ep, cmd.pack(), timeout=timeout_ms)
        assert l == 32

        if data:
            l = self.usb_dev.write(self.cmd_ep, data, timeout=timeout_ms)
            assert l == len(data)

        data = self.usb_dev.read(self.rsp_ep, Response._STRUCT.size,
                                 timeout=timeout_ms)
        assert len(data) == Response._STRUCT.size
        rsp = Response.unpack(data)
        assert rsp.opcode == opcode
        assert rsp.tag    == tag

        if rsp.data_len:
            rx_data = bytes(self.usb_dev.read(self.rsp_ep, rsp.data_len,
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
                self.usb_dev.write(self.cmd_ep, b'\x00', timeout=100)
            else:
                self.usb_dev.write(self.cmd_ep, b'', timeout=100)
            data = self.usb_dev.read(self.rsp_ep, 32, timeout=100)
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
        self.usb_dev.read(self.rsp_ep, self.max_rsp_data_len, timeout=100)
        try:
            self.usb_dev.read(self.rsp_ep, self.max_rsp_data_len, timeout=100)
        except usb.core.USBTimeoutError:
            pass

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
