#!/usr/bin/env python3
# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import argparse
import time

import xtalx.z_sensor


def date_str(posix_time):
    return '%s (%s)' % (posix_time, time.asctime(time.localtime(posix_time)))


def print_info_tcsc_u5(tc2):
    print('     Serial Num: %s' % tc2.serial_num)
    print('       Git SHA1: %s' % tc2.git_sha1)
    print('     FW Version: 0x%04X' % tc2.fw_version)
    print('           HCLK: %u' % tc2.ginfo.hclk)
    print('           DCLK: %u' % tc2.ginfo.dclk)
    print('           ACLK: %u' % tc2.ginfo.aclk)
    print('            HSE: %u MHz' % tc2.ginfo.f_hs_mhz)
    print('DV Nominal Freq: %u Hz' % tc2.ginfo.dv_nominal_hz)
    print('    Max cmd len: %u' % tc2.ginfo.cmd_buf_len)
    print('       # resets: %u' % tc2.ginfo.nresets)
    print('     Drive Type: %s' % tc2.ginfo.get_drive_type())
    print('   Reset Reason: %s' % tc2.ginfo.reset_reason)
    print('      Reset CSR: 0x%08X' % tc2.ginfo.reset_csr)
    print('      Reset SR1: 0x%08X' % tc2.ginfo.reset_sr1)
    print('Max sweep freqs: %u' % tc2.ginfo.max_sweep_entries)

    print('-----------------------')
    if not tc2.ginfo.electronics_cal_date:
        print('No electrical calibration present.')
    else:
        vdda = tc2.adc_to_v(tc2.ADC_MAX / 2)
        print('Electrical calibration:')
        print(' Calibration Date: %s' %
              date_str(tc2.ginfo.electronics_cal_date))
        print('         R Source: %f' % tc2.einfo.r_source)
        print('       R Feedback: %f' % tc2.einfo.r_feedback)
        print('         DAC-to-V: %f + %f*x' % (tc2.einfo.dac_to_v_coefs[0],
                                                tc2.einfo.dac_to_v_coefs[1]))
        print('         ADC-to-V: %f + %f*x' % (tc2.einfo.adc_to_v_coefs[0],
                                                tc2.einfo.adc_to_v_coefs[1]))
        print('     Nominal VDDA: %.2f' % vdda)

        a_dac = tc2.dac_to_a(tc2.DAC_MAX)
        print('               G0: %f' % (a_dac / vdda))

    print('-----------------------')
    if not tc2.ginfo.crystal_cal_date:
        print('No tuning fork calibration present.')
    else:
        print('Tuning fork calibration:')
        print(' Calibration Date: %s' % date_str(tc2.ginfo.crystal_cal_date))
    if tc2.ginfo.cal_dac_amplitude:
        v = tc2.dac_to_a(tc2.ginfo.cal_dac_amplitude)
        if v is not None:
            print('Cal DAC Amplitude: %u (%.2fV)' %
                  (tc2.ginfo.cal_dac_amplitude, v))
        else:
            print('Cal DAC Amplitude: %u' % tc2.ginfo.cal_dac_amplitude)
    else:
        print('Cal DAC Amplitude: Not present')


def main(rv):
    devs = xtalx.z_sensor.find(serial_number=rv.sensor)
    for dev in devs:
        tc2 = xtalx.z_sensor.make(dev)
        print('*************************************')
        if isinstance(tc2, xtalx.z_sensor.TCSC_U5):
            print_info_tcsc_u5(tc2)
            continue

        print('Unrecognized TinCan type.')


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor')
    rv = parser.parse_args()

    try:
        main(rv)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    _main()
