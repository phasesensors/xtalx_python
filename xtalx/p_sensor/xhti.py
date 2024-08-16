# Copyright (c) 2022-2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time

from xtalx.tools.math import PolynomialFit1D, PolynomialFit2D, hex_s_to_double

import xtalx.tools.serial


class XHTICommandException(Exception):
    def __init__(self, rsp):
        super().__init__('Unexpected response: "%s"' % rsp)
        self.rsp = rsp


class Measurement:
    def __init__(self, sensor, time_ns, pressure_psi, temp_c, pressure_freq,
                 temp_freq, lse_freq, clkcal_freq):
        self.sensor        = sensor
        self.time_ns       = time_ns
        self.pressure_psi  = pressure_psi
        self.temp_c        = temp_c
        self.pressure_freq = pressure_freq
        self.temp_freq     = temp_freq
        self.lse_freq      = lse_freq
        self.clkcal_freq   = clkcal_freq

    @staticmethod
    def _from_packet(sensor, time_ns, t_count, p_count, l_count, c_count):
        t_freq = sensor.refclk_hz * 26200 / t_count if t_count else 0
        p_freq = sensor.refclk_hz * 5000  / p_count if p_count else 0
        l_freq = sensor.refclk_hz * 3272  / l_count if l_count else 0
        c_freq = c_count * 32768 / 3272

        if sensor.poly_temp and t_freq:
            temp_c = sensor.poly_temp(t_freq)
        else:
            temp_c = None

        if sensor.poly_psi and p_freq and t_freq:
            psi = sensor.poly_psi(p_freq, t_freq)
        else:
            psi = None

        return Measurement(sensor, time_ns, psi, temp_c, p_freq, t_freq, l_freq,
                           c_freq)

    def tostring(self, verbose=False):
        s = '%s: ' % self.sensor
        if verbose:
            s += ('pf %s tf %s p %s t %s' %
                  (self.pressure_freq, self.temp_freq,
                   self.pressure_psi, self.temp_c))
        else:
            if self.pressure_psi is None:
                p = 'n/a'
            else:
                p = '%f' % self.pressure_psi

            if self.temp_c is None:
                t = 'n/a'
            else:
                t = '%f' % self.temp_c
            s += '%s PSI, %s C' % (p, t)

        return s

    def to_influx_point(self, time_ns=None, measurement='xtalx_data',
                        fields=None):
        time_ns = time_ns or time.time_ns()
        fields  = fields or {}
        p = {
            'measurement' : measurement,
            'time'        : time_ns,
            'tags'        : {'sensor' : self.sensor.serial_num},
            'fields'      : fields,
        }
        if self.pressure_psi is not None:
            fields['pressure_psi'] = float(self.pressure_psi)
        if self.temp_c is not None:
            fields['temp_c'] = float(self.temp_c)
        if self.pressure_freq is not None:
            fields['pressure_freq_hz'] = float(self.pressure_freq)
        if self.temp_freq is not None:
            fields['temp_freq_hz'] = float(self.temp_freq)
        return p


class XHTI:
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
        self.refclk_hz      = None
        self.report_id      = None
        self.bias           = None
        self.p_startup_ms   = None
        self.poly_psi       = None
        self.poly_temp      = None
        self.startup_err    = None
        self._halt_yield    = True

        self._reset_and_synchronize()

    def __str__(self):
        return 'XHTI(%s)' % self.serial_num

    def _write(self, data):
        for c in data:
            self.serial.write(b'%c' % c)
            self.serial.flush()
            time.sleep(7 / 57600)

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
            raise XHTICommandException(l)
        return l

    def _exec_long_command(self, cmd):
        self._write(cmd + b'\r')
        rsp = []
        while True:
            l = self._readline()
            if l == b'#\r\n':
                raise XHTICommandException(rsp)
            if l == b'=\r\n':
                return rsp

            rsp.append(l)

            if self.fw_version < 0x210 and l.startswith(b'E: '):
                raise XHTICommandException(rsp)

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

    def _handle_F_line(self, l):
        # Nothing to do.
        pass

    def _handle_P_line(self, l):
        # Handle the PLL line.
        assert l.startswith(b'P: Max PLL @ ')
        _, _, r = l.partition(b' @ ')
        words   = r.split()
        assert words[1] == b'Hz'
        self.refclk_hz = int(words[0])

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
        self.refclk_hz      = None
        self.report_id      = None
        self.bias           = None
        self.p_startup_ms   = None
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
            elif l.startswith(b'F: '):
                self._handle_F_line(l)
            elif l.startswith(b'P: '):
                self._handle_P_line(l)
            elif l.startswith(b'e: '):
                self._handle_e_line(l)

        # Get calibration info.
        hdr = self.read_calibration_header()
        _, self.report_id, self.bias, self.p_startup_ms = hdr

        # Finally, read polynomials.
        self.poly_psi  = self.read_pressure_polynomial()
        self.poly_temp = self.read_temperature_polynomial()

    def read_calibration_header(self):
        '''
        Returns the tuple (hse_freq_x10, report_id, bias, p_startup_ms).
        '''
        if self.fw_version < 0x216:
            rsp = self._exec_command(b'HDR')
        else:
            rsp = self._exec_long_command(b'HDR')
            assert len(rsp) == 1
            rsp = rsp[0]
        words = rsp[3:].split()

        d = dict(zip(words[::2], words[1::2]))
        for k in d:
            if k not in (b'RefClk', b'Id', b'Bias', b'PStartupMs', b'PLLClk'):
                raise Exception('Unrecognized response: %s' % rsp)

        ref_clk      = float(d[b'RefClk'])
        report_id    = int(d[b'Id'])
        p_startup_ms = int(d[b'PStartupMs'])
        bias         = int(d.get(b'Bias', 12053700))
        pll_clk      = int(d.get(b'PLLClk', self.refclk_hz))
        assert pll_clk == self.refclk_hz

        return ref_clk, report_id, bias, p_startup_ms

    def read_pressure_polynomial(self):
        try:
            lines = self._exec_long_command(b'PLP')
        except XHTICommandException:
            return None

        assert len(lines) >= 3
        order    = len(lines) - 3
        words    = lines[0].split(b',')
        x_domain = (hex_s_to_double(words[0]), hex_s_to_double(words[1]))
        words    = lines[1].split(b',')
        y_domain = (hex_s_to_double(words[0]), hex_s_to_double(words[1]))
        coefs    = []
        for i in range(order + 1):
            words = lines[2 + i].split(b',')
            assert len(words) == order + 1
            coefs = coefs + [hex_s_to_double(w) for w in words]
        return PolynomialFit2D.from_domain_coefs(x_domain, y_domain, coefs)

    def read_temperature_polynomial(self):
        try:
            lines = self._exec_long_command(b'PLT')
        except XHTICommandException:
            return None

        assert len(lines) == 2
        words    = lines[0].split(b',')
        x_domain = (hex_s_to_double(words[0]), hex_s_to_double(words[1]))
        words    = lines[1].split(b',')
        coefs    = [hex_s_to_double(w) for w in words]
        return PolynomialFit1D.from_domain_coefs(x_domain, coefs)

    def yield_measurements(self):
        self.serial.flushInput()
        self._write(b'CAL\r')
        self._halt_yield = False
        while not self._halt_yield:
            l = self._readline()
            if not l.startswith(b'M: '):
                continue

            time_ns = time.time_ns()

            words = l.split()
            if (words[1][0] != ord('T') or words[2][0] != ord('P') or
                    words[3][0] != ord('L') or words[4][0] != ord('C')):
                continue

            t_count = int(words[1][1:], 16)
            p_count = int(words[2][1:], 16)
            l_count = int(words[3][1:], 16)
            c_count = int(words[4][1:], 16)
            if t_count == 0xFFFFFFFF:
                t_count = 0
            if p_count == 0xFFFFFFFF:
                p_count = 0
            if l_count == 0xFFFFFFFF:
                l_count = 0
            if c_count == 0xFFFFFFFF:
                c_count = 0

            yield Measurement._from_packet(self, time_ns, t_count, p_count,
                                           l_count, c_count)
