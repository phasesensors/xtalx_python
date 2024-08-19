# Copyright (c) 2022 by Phase Advanced Sensor Systems.
# All rights reserved.
import argparse
import time
import os

import xtalx.p_sensor
from . import xmhti_qspi_decode


def measure_continuous(xmhti):
    print('Starting continuous mode...')
    xmhti.start_continuous_mode()
    while True:
        ms = xmhti.read_measurements()
        for m in ms:
            print('I:0x%08X  T:%10.3f  P:%9.3f  VDD:%5.3f  '
                  'VBUS/2:%5.3f' % (m.seq, m.ft, m.fp, m.vdd, m.vbus))
        time.sleep(9.9)


def main(args):
    d = xtalx.p_sensor.find_one_xmhti(serial_number=args.serial_number)
    xmhti = xtalx.p_sensor.make_xmhti(d)

    # Get boot status.
    bs = xmhti._boot_status
    ac = xmhti.get_autonomous_config()
    print(' Serial Num: %s' % xmhti.serial_num)
    print(' FW Version: %s' % xmhti.fw_version_str)
    print('       SHA1: %s' % xmhti.git_sha1)
    print('POST Status: 0x%08X' % bs.post_result)
    print('   PLL Freq: %s MHz' % (bs.pll_freq / 1000000))
    print('   HSE Freq: %s MHz' % (bs.hse_freq / 1000000))
    print('    LS Freq: %.3f KHz (%u ticks)' % (bs.ls_freq / 1000, bs.ls_ticks))
    if xmhti.cal_page is not None:
        print('OSC Startup: %u ms' % xmhti.cal_page.osc_startup_time_ms)
    print('Auto Config: %u secs, flags 0x%02X' % (ac.interval_secs, ac.flags))

    # Issue a Get QSPI Flash Info command.
    qi = xmhti.get_qspi_flash_info()
    print('QSPI Dev ID: 0x%02X' % qi.dev_id)
    print('QSPI Man ID: 0x%02X' % qi.m_id)
    print('  QSPI D_ID: 0x%02X' % qi.d_id)
    print('   QSPI Pos: %u / %u' % (qi.write_index, qi.nslots))
    print('  QSPI Size: %s MB'  % (qi.capacity / (1024 * 1024)))

    # Issue a Get Voltages command.
    vs = xmhti.get_voltages()
    print('        VDD: %.3f V' % (vs.vdd_mv / 1000))
    print('       VUSB: %.3f V' % (vs.vbus_mv * 2 / 1000))

    # If requested, perform a chip erase.
    if args.chip_erase_start:
        print('Sending Chip Erase command...')
        xmhti.start_erase_qspi_flash()

    # If requested, wait for a chip erase to complete.
    if args.chip_erase_wait_done:
        print('Waiting for Chip Erase to complete...')
        t0 = time.time()
        while True:
            qes = xmhti.get_qspi_erase_status()
            if not qes.erase_in_progress:
                break

        t1 = time.time()
        print('Erase completed in %.2f seconds (ER Reg=0x%02X).' %
              (t1 - t0, qes.extended_read_register))

    # If requested, store a timestamp.
    if args.timestamp:
        t = xmhti.record_timestamp()
        print('Wrote timestamp %u.' % t)

    # If necessary, read the flash log.
    if args.read_flash_log:
        print('Reading flash log...')
        t0 = time.time()
        flash_log_data = xmhti.read_flash_log()
        dt = time.time() - t0
        if dt > 0:
            print('Read %u bytes in %.2f seconds (%.2f K/s).'
                  % (len(flash_log_data), dt, len(flash_log_data) / (1024*dt)))
        else:
            print('Read %u bytes.' % len(flash_log_data))

        with open(args.read_flash_log, 'wb') as f:
            f.write(flash_log_data)

    # If necessary, read the flash.
    if args.read_qspi or args.decode_qspi:
        print('Reading QSPI flash...')
        t0 = time.time()
        if args.read_qspi:
            qspi_data = xmhti.read_qspi_flash(0, qi.nslots)
        else:
            qspi_data = xmhti.read_qspi_flash(0, qi.write_index)
        dt = time.time() - t0
        if dt > 0:
            print('Read %u bytes in %.2f seconds (%.2f K/s).'
                  % (len(qspi_data), dt, len(qspi_data) / (1024*dt)))
        else:
            print('Read %u bytes.' % len(qspi_data))

    # If requested, write the flash contents to a file.
    if args.read_qspi:
        with open(args.read_qspi, 'wb') as f:
            f.write(qspi_data)

    # If requested, decode the flash contents.
    if args.decode_qspi:
        if not os.path.isdir(args.decode_qspi):
            raise Exception('Not a directory: %s' % args.decode_qspi)
        xmhti_qspi_decode.process_image(xmhti.serial_num, args.decode_qspi,
                                        qspi_data, bs.hse_freq, 0,
                                        xmhti.poly_psi, xmhti.poly_temp)

    # If requested, clear the chip erase journal.
    if args.clear_erase_journal:
        print('Clearing chip-erase journal...')
        xmhti.clear_erase_journal()

    # If requested, clear the autonomous config journal.
    if args.clear_auto_journal:
        print('Clearing autonomous-config journal...')
        xmhti.clear_auto_journal()

    # If requested, clear the flash log.
    if args.clear_flash_log:
        print('Clearing flash log...')
        xmhti.clear_flash_log()

    # If requested, set the autonomous configuration.
    if args.set_autonomous_config:
        print('Setting autonomous configuration...')
        words = args.set_autonomous_config.split(',')
        assert len(words) == 2
        interval_secs = int(words[0])
        flags         = int(words[1], 0)
        xmhti.set_autonomous_config(interval_secs, flags)

    # If requested, start continuous measurement mode.
    if args.measure_continuous:
        measure_continuous(xmhti)

    # If requested, start autonomous measurement mode.
    if args.measure_autonomous:
        print('Starting autonomous mode...')
        xmhti.start_autonomous_mode()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-number', '-s')
    parser.add_argument('--chip-erase-start', action='store_true')
    parser.add_argument('--chip-erase-wait-done', action='store_true')
    parser.add_argument('--timestamp', action='store_true')
    parser.add_argument('--read-qspi')
    parser.add_argument('--read-flash-log')
    parser.add_argument('--decode-qspi')
    parser.add_argument('--clear-erase-journal', action='store_true')
    parser.add_argument('--clear-auto-journal', action='store_true')
    parser.add_argument('--clear-flash-log', action='store_true')
    parser.add_argument('--measure-continuous', action='store_true')
    parser.add_argument('--measure-autonomous', action='store_true')
    parser.add_argument('--set-autonomous-config', help=('Set the autonomous '
                        'config: <interval_secs>,<flags>'))
    main(parser.parse_args())


if __name__ == '__main__':
    _main()
