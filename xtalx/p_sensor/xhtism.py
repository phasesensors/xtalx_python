# Copyright (c) 2022-2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time
import struct

import xtalx.modbus_adapter

from .xti import Measurement
from .cal_page import CalPage


PARITY_DICT = {
    'N' : 0,
    'E' : 2,
    'O' : 3,
}


class XHTISM:
    '''
    This is a driver for the high-temperature XHTIS sensor built with Modbus
    firmware.
    '''
    def __init__(self, bus, slave_addr=0x80):
        self.bus          = bus
        self.slave_addr   = slave_addr
        self._halt_yield  = True
        self.last_time_ns = 0

        (self.serial_num,
         self.fw_version_str,
         self.fw_version,
         self.git_sha1) = self._read_ids()

        self.cal_page = self.read_valid_calibration_page()

        self.report_id = None
        self.poly_psi  = None
        self.poly_temp = None
        if self.cal_page is not None:
            self.report_id = self.cal_page.get_report_id()
            self.poly_psi, self.poly_temp = self.cal_page.get_polynomials()

    def __str__(self):
        return 'XHTISM(%s)' % self.serial_num

    def _read_ids(self):
        objs = self.bus.read_device_identification(self.slave_addr, 0x01, 0x00)

        assert objs[0].value == b'Phase'
        assert objs[1].value.startswith(b'XHTIS-')

        fw_version_str = objs[2].value.decode()
        parts          = fw_version_str.split('.')
        fw_version = ((int(parts[0]) << 8) |
                      (int(parts[1]) << 4) |
                      (int(parts[2]) << 0))

        git_sha1 = self.bus.read_holding_registers_binary(self.slave_addr,
                                                          0x1000, 23)

        return (objs[1].value.decode(), fw_version_str, fw_version,
                git_sha1.decode().strip())

    def set_comm_params(self, baud_rate, slave_addr, parity):
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
        self.bus.write_holding_registers_binary(self.slave_addr, 0x1001,
                                                bytes([baud_rate, slave_addr,
                                                       PARITY_DICT[parity],
                                                       0x00]))

    def get_coefficients(self):
        '''
        Returns the T and P IIR filter coefficients currently in-use by the
        sensor.  These values are saved on the sensor in nonvolatile storage.
        '''
        rsp = self.bus.read_holding_registers_binary(self.slave_addr, 0x1002, 1)
        t_c = rsp[0]
        p_c = rsp[1]
        return t_c, p_c

    def set_coefficients(self, t_c, p_c):
        '''
        Set the T and P IIR filter coefficients used by the sensor.  These
        values are saved on the sensor in nonvolatile storage.  The new values
        take effect immediately; no sensor reset required.
        '''
        assert t_c < 32
        assert p_c < 32
        self.bus.write_holding_registers_binary(self.slave_addr, 0x1002,
                                                bytes([t_c, p_c]))

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for the single calibration page stored in
        flash.
        '''
        data = b''
        for i in range(CalPage.get_short_size() // 8):
            address = 0x2000 + i*4
            data += self.bus.read_holding_registers_binary(self.slave_addr,
                                                           address, 4)
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

    def read_measurement(self):
        rsp = self.bus.read_holding_registers_binary(self.slave_addr, 0, 8)

        ft, fp = struct.unpack('<dd', rsp)

        psi = None
        if self.poly_psi is not None and ft and fp:
            psi = self.poly_psi(fp, ft)

        temp_c = None
        if self.poly_temp is not None and ft:
            temp_c = self.poly_temp(ft)

        m = Measurement(self, None, psi, temp_c, fp, ft, None, None,
                        None, None, None, None, None, None, None)
        if isinstance(self.bus, xtalx.modbus_adapter.MBA):
            m._current_amps = self.bus.measure_current()

        return m

    def yield_measurements(self, poll_interval_sec=0.1, **_kwargs):
        self._halt_yield = False
        while not self._halt_yield:
            yield self.read_measurement()
            time.sleep(poll_interval_sec)

    def halt_yield(self):
        self._halt_yield = True

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
