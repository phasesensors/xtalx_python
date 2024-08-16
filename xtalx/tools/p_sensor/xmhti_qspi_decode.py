# Copyright (c) 2022 by Phase Advanced Sensor Systems.
# All rights reserved.
from datetime import datetime, timezone
import argparse
import struct
import math
import os

import numpy as np


SS_NOP              = 0
SS_TEXT             = 1
SS_USB_STARTUP      = 2
SS_BATTERY_STARTUP  = 3
SS_TIMESTAMP        = 4
SS_AUTONOMOUS_START = 5
SS_PT_FREQS         = 6
SS_TPLC_COUNTS      = 7
SS_TPL_FREQS        = 8

BLANK_SLOT = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF'


class Record:
    def __init__(self, data):
        self.data = data


class BlankRecord(Record):
    '''
    8 bytes of 0xFF.
    '''
    def __repr__(self):
        return 'BlankRecord: FFFFFFFFFFFFFFFF'


class JunkRecord(Record):
    '''
    Data that cannot be decoded.
    '''
    def __repr__(self):
        return 'JunkRecord: %s' % self.data.hex()


class NopRecord(Record):
    def __repr__(self):
        return 'NOP: %s' % self.data.hex()


class USBStartup(Record):
    def __init__(self, data):
        super().__init__(data)
        (self.post_result,
         self.csr,
         self.sr1,
         self.reset_reason) = struct.unpack('<IIIB', data)

    def __repr__(self):
        return ('USBStartup: POST=0x%08X CSR=0x%08X SR1=0x%08X RR=0x%02X' %
                (self.post_result, self.csr, self.sr1, self.reset_reason))


class Timestamp(Record):
    def __init__(self, data):
        super().__init__(data)
        self.timestamp = struct.unpack_from('<I', data)[0]

    def __repr__(self):
        dt  = datetime.fromtimestamp(self.timestamp, timezone.utc)
        ldt = dt.astimezone(tz=None)
        return 'Timestamp: %s' % ldt.strftime('%c %Z')


class AutoStart(Record):
    def __init__(self, data):
        super().__init__(data)
        self.period_sec = struct.unpack_from('<I', data)[0]

    def __repr__(self):
        return 'AutoStart: %u sec interval' % self.period_sec


class TPLC(Record):
    def __init__(self, data, ref_freq, bias_count):
        super().__init__(data)
        self.t_count = (data[0] << 16) | (data[ 1] << 8) | data[ 2]
        self.p_count = (data[3] << 16) | (data[ 4] << 8) | data[ 5]
        self.l_count = (data[6] << 16) | (data[ 7] << 8) | data[ 8]
        self.c_count = (data[9] << 16) | (data[10] << 8) | data[11]
        self.iter    = data[12]
        self.csum    = data[13]

        self.t_hz = self.p_hz = self.l_hz = 0

        if self.t_count != 0xFFFFFF:
            self.t_count += bias_count
            self.t_hz     = ref_freq * 26200 / self.t_count
        else:
            self.t_count = 0xFFFFFFFF
        if self.p_count != 0xFFFFFF:
            self.p_count += bias_count
            self.t_hz     = ref_freq * 5000 / self.p_count
        else:
            self.p_count = 0xFFFFFFFF
        if self.l_count != 0xFFFFFF:
            self.l_count += bias_count
            self.l_hz     = ref_freq * 3272 / self.l_count
        else:
            self.l_count = 0xFFFFFFFF
        if self.c_count != 0xFFFFFF:
            self.c_count += bias_count
        else:
            self.c_count = 0xFFFFFFFF

    def __repr__(self):
        return ('TPLC(0x%02X): T%08X P%08X L%08X C%08X' %
                (self.iter, self.t_count, self.p_count, self.l_count,
                 self.c_count))


class PT(Record):
    def __init__(self, data):
        super().__init__(data)
        p_freq_csum = struct.unpack('<I', data[0:4])[0]
        self.p_freq = (p_freq_csum & 0x0FFFFFFF)
        self.csum   = (p_freq_csum >> 28)
        self.t_freq = (struct.unpack('<I', data[3:7])[0] >> 8)
        if self.p_freq in (0, 0x0FFFFFFF):
            self.p_hz = math.nan
        else:
            self.p_hz = 37000 + (16384 * self.p_freq / 2**28)
        if self.t_freq in (0, 0x00FFFFFF):
            self.t_hz = math.nan
        else:
            self.t_hz = 260952 + (4096 * self.t_freq / 2**24)

    def __repr__(self):
        return ('PT: T%.6f  P%.6f' % (self.t_hz, self.p_hz))


class TPL(Record):
    def __init__(self, data):
        super().__init__(data)
        self.rt_freq = struct.unpack('<I', data[0:4])[0]
        self.rp_freq = struct.unpack('<I', data[4:8])[0]
        self.rl_freq = struct.unpack('<I', data[8:12])[0]
        if self.rt_freq in (0, 0xFFFFFFFF):
            self.t_hz = math.nan
        else:
            self.t_hz = 260952 + (4096 * self.rt_freq / 2**32)
        if self.rp_freq in (0, 0xFFFFFFFF):
            self.p_hz = math.nan
        else:
            self.p_hz = 37000 + (16384 * self.rp_freq / 2**32)
        if self.rl_freq in (0, 0xFFFFFFFF):
            self.l_hz = math.nan
        else:
            self.l_hz = 31744 + (2048 * self.rl_freq / 2**32)
        self.iter    = data[12]
        self.csum    = data[13]

    def __repr__(self):
        return ('TPL(0x%02X): T%.6f  P%.6f  L%.6f' %
                (self.iter, self.t_hz, self.p_hz, self.l_hz))


class Unknown(Record):
    def __init__(self, op, data):
        super().__init__(data)
        self.op = op

    def __repr__(self):
        return 'Unknown(%2u): %s' % (self.op, self.data.hex())


def parse_record(data, ref_freq, bias_count):
    '''
    Parse a sequence of 8-byte slots into a record.
    '''
    record_data = b''
    for i in range(len(data) // 8):
        record_data += data[i*8 + 1:i*8 + 8]

    op = data[0] & 0x1F
    if op == SS_NOP:
        return NopRecord(record_data)
    if op == SS_USB_STARTUP and len(data) == 16:
        return USBStartup(record_data)
    if op == SS_TIMESTAMP and len(data) == 8:
        return Timestamp(record_data)
    if op == SS_AUTONOMOUS_START and len(data) == 8:
        return AutoStart(record_data)
    if op == SS_TPLC_COUNTS and len(data) == 16:
        return TPLC(record_data, ref_freq, bias_count)
    if op == SS_PT_FREQS and len(data) == 8:
        return PT(record_data)
    if op == SS_TPL_FREQS and len(data) == 16:
        return TPL(record_data)
    return Unknown(op, record_data)


class QSPIDecoder:
    def __init__(self, image, ref_freq, bias_count):
        self.image      = image
        self.ref_freq   = ref_freq
        self.bias_count = bias_count
        self.pos        = 0

    def pop_slot_data(self):
        if len(self.image) - self.pos < 8:
            return None

        slot_data = self.image[self.pos:self.pos + 8]
        self.pos += 8
        return slot_data

    def pop_record(self):
        # Get an 8-byte slot.
        slot_data = self.pop_slot_data()
        if slot_data is None:
            return None
        if slot_data == BLANK_SLOT:
            return BlankRecord(slot_data)

        # Check the slot header for the First bit.
        hdr0 = slot_data[0]
        if not hdr0 & 0x80:
            print('Missing F')
            return JunkRecord(slot_data)

        # Pop records as necessary until we see Last or something unexpected.
        data = slot_data
        hdr  = hdr0
        while not hdr & 0x40:
            # Retrieve the next slot.
            slot_data = self.pop_slot_data()
            if slot_data is None:
                return JunkRecord(data)

            # Check if record is blank.
            if slot_data == BLANK_SLOT:
                print('Missing L')
                self.pos -= 8
                return JunkRecord(data)

            # Check if First bit is set.
            hdr = slot_data[0]
            if hdr & 0x80:
                print('Redundant F')
                self.pos -= 8
                return JunkRecord(data)

            # Check if OP has changed.
            if (hdr & 0x1F) != (hdr0 & 0x1F):
                print('OP changed')
                self.pos -= 8
                return JunkRecord(data)

            # The data belongs with this record, so append it.
            data += slot_data

        # The Last bit is set, the record is complete.
        return parse_record(data, self.ref_freq, self.bias_count)


def process_image(fname, directory, data, ref_freq, bias_count,
                  poly_psi=None, poly_temp=None):
    qd             = QSPIDecoder(data, ref_freq, bias_count)
    fname          = os.path.basename(fname)
    f_num          = 1
    auto_entry     = None
    tplc_entries   = []
    tpl_entries    = []
    pt_entries     = []
    blank_rollup   = None
    last_timestamp = None
    while True:
        r = qd.pop_record()

        if isinstance(r, TPLC):
            tplc_entries.append(r)
            continue

        if isinstance(r, TPL):
            tpl_entries.append(r)
            continue

        if isinstance(r, PT):
            pt_entries.append(r)
            continue

        if isinstance(r, NopRecord):
            continue

        if tplc_entries:
            period = 0 if auto_entry is None else auto_entry.period_sec
            f_name = os.path.join(directory, fname + '.tplc.%u.csv' % f_num)
            f_num += 1
            with open(f_name, 'w', encoding='utf8') as f:
                f.write('iter,t,t_count,p_count,l_count,c_count,'
                        't_hz,p_hz,l_hz\n')
                for i, tplc in enumerate(tplc_entries):
                    f.write('%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.6f\n' %
                            (tplc.iter, i * period, tplc.t_count, tplc.p_count,
                             tplc.l_count, tplc.c_count, tplc.t_hz, tplc.p_hz,
                             tplc.l_hz))
            print('     TPLC: %u 16-byte entries written to %s' %
                  (len(tplc_entries), f_name))
            tplc_entries = []

        if tpl_entries:
            period = 0 if auto_entry is None else auto_entry.period_sec
            f_name = os.path.join(directory, fname + '.tpl.%u.csv' % f_num)
            f_num += 1
            with open(f_name, 'w', encoding='utf8') as f:
                f.write('iter,t,t_hz,p_hz,l_hz\n')
                for i, tpl in enumerate(tpl_entries):
                    f.write('%u,%u,%.6f,%.6f,%.6f\n' %
                            (tpl.iter, i * period, tpl.t_hz, tpl.p_hz,
                             tpl.l_hz))
            print('     TPL: %u 16-byte entries written to %s' %
                  (len(tpl_entries), f_name))
            tpl_entries = []

        if pt_entries:
            if auto_entry is not None:
                period = auto_entry.period_sec or 0.1
            else:
                period = 0
            f_name = os.path.join(directory, fname + '.pt.%u.csv' % f_num)
            f_num += 1
            with open(f_name, 'w', encoding='utf8') as f:
                hdrs = 't,t_hz,p_hz,temp_c,psi'
                f.write(hdrs + '\n')
                t_hzs = np.array([pt.t_hz for pt in pt_entries])
                p_hzs = np.array([pt.p_hz for pt in pt_entries])
                temp_cs = None
                psis = None
                if poly_temp is not None:
                    temp_cs = poly_temp(t_hzs)
                if poly_psi is not None:
                    psis = poly_psi(p_hzs, t_hzs)
                for i in range(len(pt_entries)):
                    t_hz   = t_hzs[i]
                    p_hz   = p_hzs[i]
                    temp_c = temp_cs[i] if temp_cs is not None else None
                    psi    = psis[i] if psis is not None else None
                    line = '%.1f,%.6f,%.6f' % (i * period, t_hz, p_hz)
                    if temp_c is not None:
                        line += ',%.4f' % temp_c
                    else:
                        line += ','
                    if psi is not None:
                        line += ',%.5f' % psi
                    else:
                        line += ','
                    f.write(line + '\n')
            print('     PT: %u 8-byte entries written to %s' %
                  (len(pt_entries), f_name))
            pt_entries = []

        if isinstance(r, AutoStart):
            auto_entry = r

        if isinstance(r, BlankRecord):
            blank_rollup = 1 if blank_rollup is None else blank_rollup + 1
            continue

        if blank_rollup is not None:
            print('    Blank: %u 8-byte entries' % blank_rollup)
            blank_rollup = None

        if r is None:
            break

        if isinstance(r, Timestamp):
            if last_timestamp is not None:
                dt = r.timestamp - last_timestamp.timestamp
                print('%s (+%u seconds)' % (r, dt))
            else:
                print(r)
            last_timestamp = r
        else:
            print(r)


def main(args):
    if not os.path.isdir(args.directory):
        raise Exception('Not a directory: %s' % args.directory)
    with open(args.file, 'rb') as f:
        data = f.read()
    process_image(args.file, args.directory, data, args.ref_freq,
                  args.bias_count)


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True)
    parser.add_argument('--directory', '-d', required=True)
    parser.add_argument('--ref-freq', '-r', type=int, default=159600000)
    parser.add_argument('--bias-count', '-b', type=int, default=12053700)
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
