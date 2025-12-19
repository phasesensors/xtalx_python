# Copyright (c) 2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time

from . import xhtiss_091
from . import xhtiss_092
from .cal_page import CalPage


class XHTISS:
    '''
    This is a driver for the high-temperature XHTISS sensor with an SPI
    interface.
    '''
    def __init__(self, bus):
        self.bus               = bus
        self._halt_yield       = True
        self.last_time_ns      = 0
        self.poll_interval_sec = 0

        self.comms = self._probe_comms()

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
        return 'XHTISS(%s)' % self.serial_num

    def _probe_comms(self):
        # Probe the target firmware.
        tx_cmd = b'\x34\x00\x00'
        tx_cmd = tx_cmd + bytes([xhtiss_092.crc8(tx_cmd)])
        while True:
            # Send a NOP command.  This will be interpreted as an unsupported
            # command by 0.9.1 firmware.
            rsp = self.bus.transact(tx_cmd)
            if rsp == b'\xAA\xBB??':
                return xhtiss_091.Comms(self)

            # The response didn't look like 0.9.1 firmware, analyze it.
            if rsp[0] != 0xAA:
                continue
            if rsp[1] != 0x00:
                continue
            if rsp[2] != 0x34:
                continue
            if rsp[3] != xhtiss_092.crc8(rsp[0:3]):
                continue

            # The response was good for an 0.9.2 or higher firmware; we can
            # double-check the sticky status code.
            rsp = self.bus.transact(b'\x00\x00')
            if rsp[0] != 0xAA:
                continue
            if rsp[1] != 0x00:
                continue

            return xhtiss_092.Comms(self)

    def _read_ids(self):
        data = self.comms._get_nvstore(0xCA01, 24)
        if data[0] == 0xFF or data[0] == 0x00:
            raise Exception('Invalid ID response from sensor, may not be '
                            'connected or powered.')
        serial_number = data.decode().strip('\x00')

        data = self.comms._get_nvstore(0xCA02, 10)
        fw_version_str = data.decode().strip('\x00')
        parts          = fw_version_str.split('.')
        fw_version = ((int(parts[0]) << 8) |
                      (int(parts[1]) << 4) |
                      (int(parts[2]) << 0))

        data = self.comms._get_nvstore(0xCA03, 48)
        git_sha1 = data.decode().strip('\x00')

        return serial_number, fw_version_str, fw_version, git_sha1

    def get_flash_params(self):
        data = self.comms._get_nvstore(0xC0EF, 4)
        t_c = data[0]
        p_c = data[1]
        sample_ms = (data[3] << 8) | (data[2] << 0)
        return t_c, p_c, sample_ms

    def set_flash_params(self, t_c, p_c, sample_ms):
        data = bytes([t_c, p_c, sample_ms & 0xFF, (sample_ms >> 8) & 0xFF])
        self.comms._set_nvstore(0xC0EF, data)

    def read_calibration_pages_raw(self):
        '''
        Returns the raw data bytes for the single calibration page stored in
        flash.
        '''
        data = b''
        for i in range(CalPage.get_short_size() // 4):
            address = 0x2000 + i*4
            data += self.comms._get_nvstore(address, 4)
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

    def yield_measurements(self, poll_interval_sec=None, **_kwargs):
        if poll_interval_sec is None:
            poll_interval_sec = self.poll_interval_sec

        self._halt_yield = False
        while not self._halt_yield:
            try:
                m = self.comms.read_measurement()
            except xhtiss_092.OpcodeMismatchError as e:
                print('Opcode mismatch: tx_cmd "%s" data "%s"' %
                      (e.tx_cmd.hex(), e.data.hex()))
                continue
            except xhtiss_092.RXChecksumError as e:
                print('RX checksum error: tx_cmd "%s" data "%s" exp 0x%02X' %
                      (e.tx_cmd.hex(), e.data.hex(), e.exp_csum))
                continue
            if m._age_ms > 25:
                continue

            yield m
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
