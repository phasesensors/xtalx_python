# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
from enum import IntEnum
import errno

import btype
import usb

import xtalx.usbcmd


CMD_EP       = 0x01
RSP_EP       = 0x81
TELEMETRY_EP = 0x82


class Opcode(IntEnum):
    SET_SAMPLE_INTERVAL_MS  = 0x100
    SET_IIR_ALPHA_SHIFT     = 0x101
    GET_T_POLY_PARAMS       = 0x102
    SET_T_POLY_PARAMS       = 0x103
    GET_P_POLY_PARAMS       = 0x104
    SET_P_POLY_PARAMS       = 0x105
    PULSE_P_ANTENNA         = 0x106
    PULSE_T_ANTENNA         = 0x107
    SET_P_POWER             = 0x108
    SET_T_POWER             = 0x109
    SET_LHP_PID_PARAMS      = 0x10A
    GET_LHP_PID_PARAMS      = 0x10B

    ERASE_FLASH_PARAMS      = 0x2FD
    ERASE_RAM_PARAMS        = 0x2FE
    SAVE_PARAMS             = 0x2FF


class TelemetryPacket(btype.Struct, endian='<'):
    seq_num         = btype.uint32_t()
    flags           = btype.uint32_t()
    temp_hz         = btype.float64_t()
    temp_c          = btype.float64_t()
    pressure_hz     = btype.float64_t()
    pressure_psi    = btype.float64_t()
    lhp_power       = btype.float32_t()
    lhp_cP          = btype.float32_t()
    lhp_cI          = btype.float32_t()
    lhp_cD          = btype.float32_t()
    _EXPECTED_SIZE  = 56


class LHPPIDType(IntEnum):
    PID_RELATIVE    = 0
    PID_ABSOLUTE    = 1


class LHPPIDParams(btype.Struct, endian='<'):
    lhp_setpoint_hz = btype.float64_t()
    lhp_pid_kp      = btype.float64_t()
    lhp_pid_ki      = btype.float64_t()
    lhp_pid_kd      = btype.float64_t()
    lhp_pid_type    = btype.uint32_t()
    rsrv            = btype.uint32_t()
    _EXPECTED_SIZE  = 40


class Measurement:
    '''
    Object encapsulating the results of an XTI sensor measurement.  The
    following fields are defined:

        sensor - Reference to the XTI that generated the Measurement.
        temp_freq - Measured temperature crystal frequency.
        temp_c - Temperature measured in degrees Celsius.
        pressure_freq - Measured pressure crystal frequency.
        pressure_psi - Temperature-compensated pressure measured in PSI.
        flags - A set of validity and error flags.

    If the sensor is uncalibrated then temp_c and pressure_psi will be None.
    '''
    def __init__(self, sensor, seq_num, flags, temp_freq, temp_c,
                 pressure_freq, pressure_psi, lhp_power, lhp_cP, lhp_cI, lhp_cD,
                 time_ns=None):
        self.sensor             = sensor
        self.seq_num            = seq_num
        self.flags              = flags
        self.temp_freq          = temp_freq
        self.temp_c             = temp_c
        self.pressure_freq      = pressure_freq
        self.pressure_psi       = pressure_psi
        self.lhp_power          = lhp_power
        self.lhp_cP             = lhp_cP
        self.lhp_cI             = lhp_cI
        self.lhp_cD             = lhp_cD
        self.time_ns            = time_ns or sensor.time_ns_increasing()

    @staticmethod
    def _from_packet(sensor, packet):
        tp = TelemetryPacket.unpack(packet)
        return Measurement(sensor, tp.seq_num, tp.flags, tp.temp_hz, tp.temp_c,
                           tp.pressure_hz, tp.pressure_psi, tp.lhp_power,
                           tp.lhp_cP, tp.lhp_cI, tp.lhp_cD)

    def tostring(self, verbose=False):
        s = '%s: ' % self.sensor
        if verbose:
            s += ('F 0x%04X tf %s t %s pf %s p %s' %
                  (self.flags, self.temp_freq, self.temp_c, self.pressure_freq,
                   self.pressure_psi))
        else:
            s += '%s PSI, %s C' % (self.pressure_psi, self.temp_c)

        return s

    def to_stsdb_point(self, time_ns=None):
        time_ns = time_ns or self.time_ns
        p = {
            'time_ns'                   : time_ns,
            'pressure_psi'              : self.pressure_psi,
            'temp_c'                    : self.temp_c,
            'pressure_freq_hz'          : self.pressure_freq,
            'temp_freq_hz'              : self.temp_freq,
            'lores_pressure_psi'        : None,
            'lores_temp_c'              : None,
            'lores_pressure_freq_hz'    : None,
            'lores_temp_freq_hz'        : None,
        }
        return p

    def to_lhp2_stsdb_point(self, time_ns=None):
        time_ns = time_ns or self.time_ns
        return {
            'time_ns'   : time_ns,
            'cP'        : self.lhp_cP,
            'cI'        : self.lhp_cI,
            'cD'        : self.lhp_cD,
            'power'     : self.lhp_power,
        }


class XTI15(xtalx.usbcmd.Device):
    def __init__(self, usb_dev):
        super().__init__(usb_dev, CMD_EP, RSP_EP, 256, 256, git_sha1_index=6,
                         default_configuration=0x68)

        self._halt_yield = True

    def __str__(self):
        return 'XTI(%s)' % self.serial_num

    def set_p_oscillator_power(self, enabled):
        '''
        Enables or disables power to the P oscillator.
        '''
        print('Setting P power enabled: %s' % enabled)
        self._exec_command(Opcode.SET_P_POWER, [int(enabled)])

    def pulse_p_antenna(self, N=10000):
        '''
        Pulse the P antenna N times in an attempt to kickstart the oscillator.
        '''
        print('Sending command to pulse P antenna (%u).' % N)
        self._exec_command(Opcode.PULSE_P_ANTENNA, [N])

    def set_t_oscillator_power(self, enabled):
        '''
        Enables or disables power to the T oscillator.
        '''
        print('Setting T power enabled: %s' % enabled)
        self._exec_command(Opcode.SET_T_POWER, [int(enabled)])

    def pulse_t_antenna(self, N=10000):
        '''
        Pulse the T antenna N times in an attempt to kickstart the oscillator.
        '''
        print('Sending command to pulse T antenna (%u).' % N)
        self._exec_command(Opcode.PULSE_T_ANTENNA, [N])

    def set_sample_interval_ms(self, ms):
        '''
        Sets the telemetry sample interval in milliseconds.  This is how
        frequently a new telemetry packet will be generated.  The new value is
        only stored in the RAM parameter copy.
        '''
        self._exec_command(Opcode.SET_SAMPLE_INTERVAL_MS, [ms])

    def set_iir_alpha_shift(self, t_iir_alpha_shift, p_iir_alpha_shift):
        '''
        Sets the shift values for the T and P IIR filters.  The alpha value for
        the filter is equal to:

            alpha = 2 ** (-alpha_shift)

        That is, the shift values are a right-shift amount for the IIR filter
        fixed-point representation.  The default values are 11 for the T filter
        and 9 for the P filter.

        The new values are only stored in the RAM parameter copy.
        '''
        shift = (p_iir_alpha_shift << 8) | (t_iir_alpha_shift)
        self._exec_command(Opcode.SET_IIR_ALPHA_SHIFT, [shift])

    def set_lhp_pid_params(self, setpoint_hz, pid_type, kP, kI, kD):
        '''
        Sets the target LHP frequency in Hz, the PID type (relative or absolute)
        and sets the coefficients for the PID loop.  The ne values are only
        stored in the RAM parameter copy.
        '''
        params = LHPPIDParams(
                lhp_setpoint_hz=setpoint_hz,
                lhp_pid_kp=kP,
                lhp_pid_ki=kI,
                lhp_pid_kd=kD,
                lhp_pid_type=pid_type,
                rsrv=0)
        self._exec_command(Opcode.SET_LHP_PID_PARAMS, data=params.pack())

    def get_lhp_pid_params(self):
        '''
        Returns the LHP PID parameters from the RAM parameters.
        '''
        _, data = self._exec_command(Opcode.GET_LHP_PID_PARAMS)
        return LHPPIDParams.unpack(data)

    def erase_flash_params(self):
        '''
        Erases the parameters stored in flash by clearing the flash log.  This
        does not affect the RAM copy of flash parameters which is what the
        sensor uses for all of its runtime operations.  The next time the
        sensor is power-cycled, it will detect the blank flash parameters and
        use default values.
        '''
        self._exec_command(Opcode.ERASE_FLASH_PARAMS, [0xC0CAC01A])

    def erase_ram_params(self):
        '''
        Resets the RAM copy of the sensor parameters to the defaults.  This
        resets the filter IIR alpha values, disables LHP support and resets the
        telemetry sample interval.  The LHP heater is turned off.
        '''
        self._exec_command(Opcode.ERASE_RAM_PARAMS, [0xC1E0D0D0])

    def save_params(self):
        '''
        Burns the RAM copy of the sensor parameters into flash so that they will
        be used automatically if/when the sensor is power-cycled.
        '''
        self._exec_command(Opcode.SAVE_PARAMS, [0xCDEDBDBE])

    def read_measurement(self, timeout=2000):
        '''
        Synchronously read a single measurement from the sensor, blocking if no
        measurement is currently available.
        '''
        p = self.usb_dev.read(TELEMETRY_EP, 64, timeout=timeout)
        return Measurement._from_packet(self, p)

    def _yield_measurements(self, _do_reset, timeout):
        # Always discard teh first measurement.
        self.read_measurement(timeout=timeout)

        while not self._halt_yield:
            try:
                yield self.read_measurement(timeout=timeout)
            except usb.core.USBError as e:
                if e.errno != errno.ETIMEDOUT:
                    raise
                continue

    def yield_measurements(self, do_reset=True, timeout=2000):
        '''
        Yields Measurement objects synchronously in the current thread,
        blocking while waiting for new measurements to be acquired.
        '''
        self._halt_yield = False
        yield from self._yield_measurements(do_reset, timeout=timeout)

    def halt_yield(self):
        '''
        Halts an ongoing yield_measurements() call, causing it to eventually
        terminate the generator loop.
        '''
        self._halt_yield = True
