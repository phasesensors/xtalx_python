# Copyright (c) 2025 Phase Advanced Sensor Systems, Inc.
from enum import IntEnum
import platform
import random

import usb
import usb.util
import btype

import xtalx.tools.modbus


class Feature(IntEnum):
    CURRENT_MEASUREMENT     = (1 << 0)


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
    SET_BAUD_RATE            = 0x0002
    XACT                     = 0x0003
    MEASURE_CURRENT          = 0x0005
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


PARITY_DICT = {
    'N' : 0x4E4F4E45,
    'E' : 0x4556454E,
    'O' : 0x4F444420,
}


class MBA(xtalx.tools.modbus.Bus):
    CMD_EP = 0x01
    RSP_EP = 0x82

    def __init__(self, usb_dev, baud_rate=115200, parity='E'):
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

        self.features = 0
        if self.serial_num.startswith('MBA-2'):
            if self.fw_version >= 0x092:
                self.features |= Feature.CURRENT_MEASUREMENT

        self._synchronize()
        self.set_comm_params(baud_rate, parity)

    def __str__(self):
        return 'MBA(%s)' % self.serial_num

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
        self._set_configuration(0x80)

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

    def set_comm_params(self, baud_rate, parity=None):
        params = [baud_rate, 0, 0, PARITY_DICT.get(parity, 0)]
        return self._exec_command(Opcode.SET_BAUD_RATE, params)

    def measure_current(self):
        if (self.features & Feature.CURRENT_MEASUREMENT) == 0:
            return 0.0

        rsp, _ = self._exec_command(Opcode.MEASURE_CURRENT)
        adc_iir = (((rsp.params[1] << 32) & 0xFFFFFFFF00000000) |
                   ((rsp.params[0] <<  0) & 0x00000000FFFFFFFF))
        return (3.3 / (4096 * 1000 * 0.033 * 2**52)) * adc_iir

    def enter_dfu_mode(self):
        return self._exec_command(Opcode.JUMP_BOOTLOADER, [0xA47B39FE])

    def transact(self, addr, data, response_time_ms, nbytes=None):
        try:
            _, rx_data = self._exec_command(Opcode.XACT,
                                            [addr, response_time_ms], data,
                                            timeout_ms=(response_time_ms + 100))
        except CommandException as e:
            if e.rsp.status != Status.XACT_FAILED:
                raise

            if e.rsp.params[0] == 0:
                raise xtalx.tools.modbus.ResponseOverflowException(b'')
            if e.rsp.params[0] == 1:
                raise xtalx.tools.modbus.ResponseUnderflowException(b'')
            if e.rsp.params[0] == 2:
                raise xtalx.tools.modbus.BadCRCException(b'', None)
            if e.rsp.params[0] == 3:
                raise xtalx.tools.modbus.BadAddressException(b'')
            if e.rsp.params[0] == 4:
                raise xtalx.tools.modbus.BabbleException(b'')
            if e.rsp.params[0] == 5:
                raise xtalx.tools.modbus.ResponseInterruptedException(b'')
            if e.rsp.params[0] == 6:
                raise xtalx.tools.modbus.ResponseFramingErrorException(b'')
            if e.rsp.params[0] == 7:
                raise xtalx.tools.modbus.ResponseTimeoutException(b'')

            raise

        # Check the response function code.
        if rx_data[1] & 0x7F != data[0]:
            raise xtalx.tools.modbus.BadFunctionException(rx_data)
        if rx_data[1] & 0x80:
            raise xtalx.tools.modbus.ExceptionResponseException(rx_data,
                                                                rx_data[2])

        return rx_data
