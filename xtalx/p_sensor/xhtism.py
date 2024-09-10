# Copyright (c) 2022-2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time
import struct

from pymodbus.client import ModbusSerialClient

from .xti import Measurement
from .cal_page import CalPage


def regs_to_bytes(regs):
    '''
    Converts a list of 16-bit Modbus registers into a string of bytes -
    basically undoing the register list that pymodbus parsed out of the byte
    stream back into the original byte stream because no, we really don't care
    about 2-byte Modbus registers in this day and age.

    This is set up so that the returned bytes are in the exact same order that
    they were written to the serial port in - no byte-swapping on register
    boundaries or anything like that.
    '''
    return b''.join([struct.pack('>H', r) for r in regs])


class XHTISM:
    '''
    This is a driver for the high-temperature XHTIS sensor built with Modbus
    firmware.
    '''
    def __init__(self, intf, baudrate=115200, slave_addr=0x80):
        assert intf is not None

        self.intf        = intf
        self.slave_addr  = slave_addr
        self.baud_rate   = baudrate
        self._halt_yield = True

        self.client = ModbusSerialClient(intf, baudrate=baudrate, parity='E')
        self.client.connect()

        (self.serial_num,
         self.fw_version_str,
         self.fw_version,
         self.git_sha1) = self._read_ids()

    def __str__(self):
        return 'XHTISM(%s)' % self.serial_num

    def _read_ids(self):
        # This doesn't always work the first time; retry it a few times.
        for _ in range(5):
            info = self.client.read_device_information(read_code=0x01,
                                                       object_id=0x00,
                                                       slave=self.slave_addr)
            if not info.isError():
                break

            time.sleep(0.1)

        assert not info.isError()
        assert info.information[0] == b'Phase'
        assert info.information[1].startswith(b'XHTIS-')

        fw_version_str = info.information[2].decode()
        parts          = fw_version_str.split('.')
        fw_version = ((int(parts[0]) << 8) |
                      (int(parts[1]) << 4) |
                      (int(parts[2]) << 0))

        rr = self.client.read_holding_registers(address=0x1000, count=23,
                                                slave=self.slave_addr)
        git_sha1 = regs_to_bytes(rr.registers)

        return (info.information[1].decode(), fw_version_str, fw_version,
                git_sha1.decode().strip())

    def set_comm_params(self, baud_rate, slave_addr):
        '''
        Set the sensor's baud rate and Modbus slave address.  The baud rate
        must be a multiple of 4800.  As per the Modbus specification, even
        parity is always used.  These values are saved on the sensor in
        nonvolatile storage.

        After the command executes on the sensor, the Modbus reply is returned
        using the original slave address and baud rate.  After the final byte
        of the reply is transmitted, the sensor resets itself and starts back
        up using the new slave address and baud rate.  Due to this reset, there
        may be an additional delay before the sensor responds to the next
        command at the new address and baud rate.

        In the event of an error storing the values to nonvolatile storage,
        the sensor returns an error code and does not reset or switch to the
        new parameters.
        '''
        assert baud_rate % 4800 == 0
        baud_rate = baud_rate // 4800
        assert baud_rate < 64
        assert slave_addr < 256
        rr = self.client.write_registers(
                address=0x1001, slave=self.slave_addr,
                values=[(baud_rate << 8) | slave_addr, (2 << 8)]
                )
        assert not rr.isError()

    def get_coefficients(self):
        '''
        Returns the T and P IIR filter coefficients currently in-use by the
        sensor.  These values are saved on the sensor in nonvolatile storage.
        '''
        rr = self.client.read_holding_registers(address=0x1002, count=1,
                                                slave=self.slave_addr)
        t_c = ((rr.registers[0] >> 8) & 0xFF)
        p_c = ((rr.registers[0] >> 0) & 0xFF)
        return t_c, p_c

    def set_coefficients(self, t_c, p_c):
        '''
        Set the T and P IIR filter coefficients used by the sensor.  These
        values are saved on the sensor in nonvolatile storage.  The new values
        take effect immediately; no sensor reset required.
        '''
        assert t_c < 32
        assert p_c < 32
        rr = self.client.write_registers(
                address=0x1002, slave=self.slave_addr,
                values=[(t_c << 8) | p_c])
        assert not rr.isError()

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for the single calibration page stored in
        flash.
        '''
        data = b''
        for i in range(CalPage.get_short_size() // 8):
            address = 0x2000 + i*4
            rr = self.client.read_holding_registers(address=address, count=4,
                                                    slave=self.slave_addr)
            data += regs_to_bytes(rr.registers)
        pad = b'\xff' * (CalPage._EXPECTED_SIZE - len(data))
        return (data + pad,)

    def read_calibration_pages(self):
        '''
        Returns a CalPage struct for the single calibration page in sensor
        flash, even if the page is missing or corrupt.
        '''
        (cp_data,) = self.read_calibration_pages_raw()
        cp = CalPage.unpack(cp_data)
        return (cp,)

    def read_valid_calibration_page(self):
        '''
        Returns CalPage struct from the sensor flash.  Returns None if the
        calibration is not present or corrupted.
        '''
        (cp,) = self.read_calibration_pages()
        return cp if cp.is_valid() else None

    def yield_measurements(self, poll_interval_sec=0.1):
        self._halt_yield = False
        while not self._halt_yield:
            rr = self.client.read_holding_registers(address=0, count=8,
                                                    slave=self.slave_addr)

            ft = regs_to_bytes(rr.registers[:4])
            fp = regs_to_bytes(rr.registers[4:8])
            ft = struct.unpack('<d', ft)[0]
            fp = struct.unpack('<d', fp)[0]

            m = Measurement(self, None, None, None, fp, ft, None, None,
                            None, None, None, None, None, None, None)
            yield m

            time.sleep(poll_interval_sec)

    def halt_yield(self):
        self._halt_yield = True
