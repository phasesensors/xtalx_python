# Copyright (c) 2025 Phase Advanced Sensor Systems, Inc.
from enum import IntEnum

import xtalx.tools.modbus
import xtalx.usbcmd


class Feature(IntEnum):
    CURRENT_MEASUREMENT     = (1 << 0)


class Status(IntEnum):
    XACT_FAILED     = 7
    XACT_FAILED_2   = 100

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            return '%s' % Status(rsp.status)
        except ValueError:
            return xtalx.usbcmd.Status.rsp_to_status_str(rsp)


class Opcode(IntEnum):
    SET_VEXT                 = 0x0001
    SET_BAUD_RATE            = 0x0002
    XACT                     = 0x0003
    MEASURE_CURRENT          = 0x0005
    JUMP_BOOTLOADER          = 0x7B42


CMD_EP = 0x01
RSP_EP = 0x82


PARITY_DICT = {
    'N' : 0x4E4F4E45,
    'E' : 0x4556454E,
    'O' : 0x4F444420,
}


class MBA(xtalx.tools.modbus.Bus,
          xtalx.usbcmd.Device):

    def __init__(self, usb_dev, baud_rate=115200, parity='E'):
        xtalx.tools.modbus.Bus.__init__(self)
        xtalx.usbcmd.Device.__init__(self, usb_dev, CMD_EP, RSP_EP, 256, 256,
                                     git_sha1_index=6,
                                     default_configuration=0x80)

        if self.fw_version < 0x093:
            self.xact_failed_code = Status.XACT_FAILED
        else:
            self.xact_failed_code = Status.XACT_FAILED_2

        self.features = 0
        if self.serial_num.startswith('MBA-2'):
            if self.fw_version >= 0x092:
                self.features |= Feature.CURRENT_MEASUREMENT

        self.set_comm_params(baud_rate, parity)

    def __str__(self):
        return 'MBA(%s)' % self.serial_num

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
        except xtalx.usbcmd.CommandException as e:
            if e.rsp.status != self.xact_failed_code:
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
