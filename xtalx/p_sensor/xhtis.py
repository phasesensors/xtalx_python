# Copyright (c) 2022-2026 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time
import struct

import xtalx.tools.serial

from .xti import Measurement
from .cal_page import CalPage


class XHTISCommandException(Exception):
    def __init__(self, rsp):
        super().__init__('Unexpected response: "%s"' % rsp)
        self.rsp = rsp


class XHTIS:
    def __init__(self, intf, baudrate=None, spy=False):
        assert intf is not None

        if baudrate is None:
            baudrate = 57600

        self.intf   = intf
        self.serial = xtalx.tools.serial.from_intf(intf, spy=spy,
                                                   baudrate=baudrate,
                                                   timeout=1, exclusive=True)

        self.fw_name        = None
        self.platform       = None
        self.fw_version_str = None
        self.fw_version     = None
        self.git_sha1       = None
        self.gcc_version    = None
        self.serial_num     = None
        self.serial_date    = None
        self.startup_err    = None
        self._halt_yield    = True
        self.last_time_ns   = 0

        self._reset_and_synchronize()

        self.cal_page = self.read_valid_calibration_page()

        self.report_id      = None
        self.poly_psi       = None
        self.poly_temp      = None
        if self.cal_page is not None:
            self.report_id = self.cal_page.get_report_id()
            self.poly_psi, self.poly_temp = self.cal_page.get_polynomials()

        assert self.fw_version >= 0x091

    def __str__(self):
        return 'XHTIS(%s)' % self.serial_num

    def _write(self, data):
        self.serial.write(data)

    def _readline(self):
        return self.serial.readline()

    def _flush_input(self):
        timeout = self.serial.timeout
        self.serial.timeout = 0
        while self.serial.read_all():
            pass
        self.serial.timeout = timeout

    def _exec_command(self, cmd):
        self._write(cmd + b'\r')

        l = self._readline()
        if not l.startswith(b'S: '):
            print('Failed: %s' % cmd)
            raise XHTISCommandException(l)
        return l

    def _exec_long_command(self, cmd):
        self._write(cmd + b'\r')
        rsp = []
        while True:
            l = self._readline()
            if l == b'#\r\n':
                raise XHTISCommandException(rsp)
            if l == b'=\r\n':
                return rsp

            rsp.append(l)

    def _reset_and_synchronize(self):
        self._flush_input()
        self._write(b'R\r')
        self._synchronize()

    def _handle_R_line(self, l):
        # Handle the Reset line.
        assert l.startswith(b'R: ')
        words               = l.split()
        self.fw_name        = words[1].decode()
        self.platform       = words[2].decode()
        self.fw_version_str = words[3].decode()

        parts           = self.fw_version_str.split('.')
        self.fw_version = ((int(parts[0]) << 8) |
                           (int(parts[1]) << 4) |
                           (int(parts[2]) << 0))

    def _handle_G_line(self, l):
        # Handle the Git SHA1 line.
        assert l.startswith(b'G: ')
        self.git_sha1 = l.split()[-1].decode()

    def _handle_c_line(self, l):
        # Handle the compiler version line.
        assert l.startswith(b'c: ')
        self.gcc_version = l[3:].decode()

    def _handle_I_line(self, l):
        # Handle the Identity line.
        assert l.startswith(b'I: ')
        words            = l.split()
        self.serial_num  = words[1].decode()
        self.serial_date = words[2].decode()

    def _handle_e_line(self, l):
        # Handle the startup error line.
        assert l.startswith(b'e: 0x')
        _, err           = l.split()
        self.startup_err = int(err, 16)

    def _synchronize(self, timeout=10):
        # Nuke everything.
        self.fw_name        = None
        self.platform       = None
        self.fw_version_str = None
        self.fw_version     = None
        self.git_sha1       = None
        self.gcc_version    = None
        self.serial_num     = None
        self.serial_date    = None
        self.report_id      = None
        self.poly_psi       = None
        self.poly_temp      = None
        self.startup_err    = 0

        # Wait for the Reset line.
        t0 = time.time()
        while True:
            l = self._readline()
            if l.startswith(b'R: '):
                break
            if time.time() - t0 >= timeout:
                raise Exception('Reset failed.')
        self._handle_R_line(l)

        # Handle all other lines.
        while True:
            l = self._readline()
            if l in (b'=\r\n', b'#\r\n'):
                break
            if l.startswith(b'G: '):
                self._handle_G_line(l)
            if l.startswith(b'c: '):
                self._handle_c_line(l)
            elif l.startswith(b'I: '):
                self._handle_I_line(l)
            elif l.startswith(b'e: '):
                self._handle_e_line(l)

    def _read_cal_page_ram(self, offset):
        '''
        Returns 16 bytes of data from the RAM copy of the calibration page.
        '''
        assert offset % 16 == 0
        rsp = self._exec_command(b'RCPR%08X' % offset)
        assert len(rsp) == 3 + 8*5 + 2
        _offset = int(rsp[3:3 + 8], 16)
        assert _offset == offset
        return bytes.fromhex(rsp[3 + 8:].decode())

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for the single calibration page stored in
        flash.
        '''
        data = b''
        for i in range(CalPage.get_short_size() // 16):
            data += self._read_cal_page_ram(i * 16)
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

    def enable_t_oscillator(self):
        self._exec_command(b'ton')

    def enable_p_oscillator(self):
        self._exec_command(b'toff')

    def disable_t_oscillator(self):
        self._exec_command(b'pon')

    def disable_p_oscillator(self):
        self._exec_command(b'poff')

    def yield_measurements(self, **_kwargs):
        self.serial.flushInput()
        self._write(b'IIR\r')
        self._halt_yield = False
        while not self._halt_yield:
            l = self._readline()
            if not l.startswith(b'm: '):
                continue
            if len(l) != 37:
                continue

            data = bytes.fromhex(l[3:35].decode())
            t_period, p_period = struct.unpack('>dd', data)

            ft = 1. / t_period if t_period else None
            fp = 1. / p_period if p_period else None

            psi = None
            if self.poly_psi is not None and ft and fp:
                psi = self.poly_psi(fp, ft)

            temp_c = None
            if self.poly_temp is not None and ft:
                temp_c = self.poly_temp(ft)

            m = Measurement(self, None, psi, temp_c, fp, ft, None, None,
                            None, None, None, None, None, None, None)
            yield m

    def halt_yield(self):
        self._halt_yield = True

    def time_ns_increasing(self):
        '''
        Returns a time value in nanoseconds that is guaranteed to increase
        after every single call.  This function is not thread-safe.
        '''
        self.last_time_ns = t = max(time.time_ns(), self.last_time_ns + 1)
        return t
