# Copyright (c) 2025-2026 Phase Advanced Sensor Systems, Inc.
from enum import IntEnum

import xtalx.usbcmd


class Feature(IntEnum):
    CURRENT_MEASUREMENT     = (1 << 0)


class Status(IntEnum):
    XACT_FAILED     = 7

    @staticmethod
    def rsp_to_status_str(rsp):
        try:
            return '%s' % Status(rsp.status)
        except ValueError:
            return xtalx.usbcmd.Status.rsp_to_status_str(rsp)


class Opcode(IntEnum):
    SET_VEXT                 = 0x0001
    SET_FREQUENCY            = 0x0002
    XACT                     = 0x0003
    MEASURE_CURRENT          = 0x0004
    JUMP_BOOTLOADER          = 0x7B42


CMD_EP = 0x01
RSP_EP = 0x82


class SPIA(xtalx.usbcmd.Device):
    def __init__(self, usb_dev):
        super().__init__(usb_dev, CMD_EP, RSP_EP, 256, 256,
                         git_sha1_index=6,
                         default_configuration=0x84)

        self.features = 0
        if self.serial_num.startswith('XSPI-2'):
            if self.fw_version >= 0x091:
                self.features |= Feature.CURRENT_MEASUREMENT

    def __str__(self):
        return 'SPIA(%s)' % self.serial_num

    def set_vext(self, enabled):
        return self._exec_command(Opcode.SET_VEXT, [enabled])

    def set_spi_frequency(self, hz):
        rsp, _ = self._exec_command(Opcode.SET_FREQUENCY, [hz])
        return rsp.params[0]

    def measure_current(self):
        if (self.features & Feature.CURRENT_MEASUREMENT) == 0:
            return 0.0

        rsp, _ = self._exec_command(Opcode.MEASURE_CURRENT)
        adc_iir = (((rsp.params[1] << 32) & 0xFFFFFFFF00000000) |
                   ((rsp.params[0] <<  0) & 0x00000000FFFFFFFF))
        return (3.3 / (4096 * 1000 * 0.033 * 2**52)) * adc_iir

    def enter_dfu_mode(self):
        return self._exec_command(Opcode.JUMP_BOOTLOADER, [0xA47B39FE])

    def transact(self, data):
        rsp, rx_data = self._exec_command(Opcode.XACT, data=data)

        assert rsp.data_len == len(data)
        assert rsp.data_len == len(rx_data)

        return rx_data
