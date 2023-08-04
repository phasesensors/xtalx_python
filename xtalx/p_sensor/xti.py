# Copyright (c) 2020-2023 by Phase Advanced Sensor Systems Corp.
import threading
import errno
import usb
import usb.util

import btype

from .exception import XtalXException


FC_FLAGS_VALID              = (1 << 15)
FC_FLAG_NO_TEMP_PRESSURE    = (1 << 4)
FC_FLAG_PRESSURE_FAILED     = (1 << 3)
FC_FLAG_TEMP_FAILED         = (1 << 2)
FC_FLAG_PRESSURE_UPDATE     = (1 << 1)
FC_FLAG_TEMP_UPDATE         = (1 << 0)


class FrequencyPacket24(btype.Struct):
    '''
    Firmware revisions 1.0.6 and earlier return a 24-byte packet if the sensor
    doesn't have enough data to perform a temperature-compensated pressure
    measurement yet or if the sensor doesn't have a calibration applied in
    flash.
    '''
    ref_freq            = btype.uint32_t()
    pressure_edges      = btype.uint32_t()
    pressure_ref_clocks = btype.uint32_t()
    temp_edges          = btype.uint32_t()
    temp_ref_clocks     = btype.uint32_t()
    flags               = btype.uint16_t()
    seq_num             = btype.uint8_t()
    rsrv                = btype.uint8_t()
    _EXPECTED_SIZE      = 24


class FrequencyPacket40(btype.Struct):
    '''
    Firmware revisions 1.0.6 and earlier return a 40-byte packet if the sensor
    has enough data to perform a temperature-compensated pressure measurement.
    '''
    ref_freq            = btype.uint32_t()
    pressure_edges      = btype.uint32_t()
    pressure_ref_clocks = btype.uint32_t()
    temp_edges          = btype.uint32_t()
    temp_ref_clocks     = btype.uint32_t()
    flags               = btype.uint16_t()
    seq_num             = btype.uint8_t()
    rsrv                = btype.uint8_t()
    pressure_psi        = btype.float64_t()
    temp_c              = btype.float64_t()
    _EXPECTED_SIZE      = 40


class FrequencyPacket56(btype.Struct):
    '''
    Firmware revisions 1.0.7 and higher always return a 56-byte packet that
    contains flags indicating the validity of things like the temperature-
    compensated pressure measurement.  These firmware versions also return the
    MCU temperature as a control.
    '''
    ref_freq            = btype.uint32_t()
    pressure_edges      = btype.uint32_t()
    pressure_ref_clocks = btype.uint32_t()
    temp_edges          = btype.uint32_t()
    temp_ref_clocks     = btype.uint32_t()
    flags               = btype.uint16_t()
    seq_num             = btype.uint8_t()
    rsrv                = btype.uint8_t()
    pressure_psi        = btype.float64_t()
    temp_c              = btype.float64_t()
    mcu_temp_c          = btype.float64_t()
    rsrv2               = btype.Array(btype.uint8_t(), 8)
    _EXPECTED_SIZE      = 56


class Measurement:
    '''
    Object encapsulating the results of an XTI sensor measurement.  The
    following fields are defined:

        sensor - Reference to the XTI that generated the Measurement.
        ref_freq - Frequency of the sensor's reference crystal.
        pressure_edges - Number of pressure crystal ticks used to generate the
            Measurement.
        pressure_ref_clocks - Number of reference clock ticks that elapsed
            while counting pressure_edges pressure crystal ticks.
        pressure_freq - Measured pressure crystal frequency.
        temp_edges - Number of temperature crystal ticks used to generate the
            Measurement.
        temp_ref_clocks - Number of temperature crystal ticks that elapsed
            while counting temp_edges temperature crystal ticks.
        temp_freq - Measured temperature crystal frequency.
        mcu_temp_c - Microcontroller's internal junction temperature.
        pressure_psi - Temperature-compensated pressure measured in PSI.
        temp_c - Temperature measured in degrees Celsius.
        flags - A set of validity and error flags.

    If the sensor is uncalibrated or has not sampled enough data to generate
    a temperature-compensated pressure measurement then some or all of
    temp_freq, pressure_freq, pressure_psi and temp_c may be None.

    The flags field is a bitmask which may include any of the following bits;
    it may be None if the firmware version predates the introduction of status
    flags:

        FC_FLAGS_VALID - The flags field contains valid information (always set
            or flags will be None).
        FC_FLAG_NO_TEMP_PRESSURE - Will be set if pressure_psi and temp_c could
            not be generated; the sensor may be uncalibrated or may not have
            generated both temperature and pressure crystal readings yet.
        FC_FLAG_PRESSURE_FAILED - Will be set if 0.5 seconds elapse without a
            pressure crystal measurement completing; this indicates that a
            sensor failure has caused the pressure crystal to stop ticking.
        FC_FLAG_TEMP_FAILED - Will be set if 0.5 seconds elapse without a
            temperature crystal measurement completing; this indicates that a
            sensor failure has caused the temperature crystal to stop ticking.
        FC_FLAG_PRESSURE_UPDATE - Indicates that the current Measurement
            incorporates a new reading from the pressure crystal; it may still
            be incorporating the previous reading from the temperature crystal.
        FC_FLAG_TEMP_UPDATE - Indicates that the current Measurement
            incorporates a new reading from the temperature crystal; it may
            still be incorporating the previous reading from the pressure
            crystal.

    Note that since the temperature and pressure crystals tick asynchronously
    with respect to one another, a measurement on one crystal is likely to
    complete while a measurement on the other crystal is still pending and so
    typically only one of FC_FLAG_PRESSURE_UPDATE or FC_FLAG_TEMP_UPDATE will
    be set.
    '''
    def __init__(self, sensor, ref_freq, pressure_edges, pressure_ref_clocks,
                 temp_edges, temp_ref_clocks, mcu_temp_c, pressure_psi,
                 temp_c, flags):
        self.sensor              = sensor
        self.ref_freq            = ref_freq
        self.pressure_edges      = pressure_edges
        self.pressure_ref_clocks = pressure_ref_clocks
        self.temp_edges          = temp_edges
        self.temp_ref_clocks     = temp_ref_clocks
        self.mcu_temp_c          = mcu_temp_c
        self.pressure_psi        = pressure_psi
        self.temp_c              = temp_c
        self.flags               = flags

        if temp_ref_clocks > 3:
            self.temp_freq = ref_freq * temp_edges / temp_ref_clocks
        else:
            self.temp_freq = None

        if pressure_ref_clocks > 3:
            self.pressure_freq = ref_freq * pressure_edges / pressure_ref_clocks
        else:
            self.pressure_freq = None

    @staticmethod
    def _from_packet(sensor, packet):
        mt, p, t = None, None, None
        if sensor.usb_dev.bcdDevice < 0x0107:
            if len(packet) == 24:
                fp = FrequencyPacket24.unpack(packet)
            else:
                fp = FrequencyPacket40.unpack(packet)
                p  = fp.pressure_psi
                t  = fp.temp_c
        else:
            fp = FrequencyPacket56.unpack(packet)
            mt = fp.mcu_temp_c
            assert fp.flags and (fp.flags & FC_FLAGS_VALID)
            if (fp.flags & FC_FLAG_NO_TEMP_PRESSURE) == 0:
                p = fp.pressure_psi
                t = fp.temp_c
        flags = fp.flags if fp.flags & FC_FLAGS_VALID else None

        return Measurement(sensor, fp.ref_freq, fp.pressure_edges,
                           fp.pressure_ref_clocks, fp.temp_edges,
                           fp.temp_ref_clocks, mt, p, t, flags)

    def tostring(self, verbose=False):
        s = '%s: ' % self.sensor
        if verbose:
            s += ('C %u pe %u prc %u pf %f te %u trc %u tf %f p %s t %s '
                  'mt %s' % (self.ref_freq, self.pressure_edges,
                             self.pressure_ref_clocks, self.pressure_freq,
                             self.temp_edges, self.temp_ref_clocks,
                             self.temp_freq, self.pressure_psi, self.temp_c,
                             self.mcu_temp_c))
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


class XTI:
    '''
    Given a USB device handle acquired via find() or find_one(), creates an
    XTI object that can be used to communicate with a sensor.
    '''
    def __init__(self, usb_dev):
        self.usb_dev     = usb_dev
        self.lock        = threading.RLock()
        self._halt_yield = True
        self.thread      = None

        try:
            self.serial_num = usb_dev.serial_number
            self.git_sha1   = usb.util.get_string(usb_dev, 6)
            self.fw_version = usb_dev.bcdDevice
        except ValueError as e:
            if str(e) == 'The device has no langid':
                raise XtalXException(
                    'Device has no langid, ensure running as root!') from e

        if self.usb_dev.bcdDevice >= 0x0103:
            try:
                self.report_id = usb.util.get_string(usb_dev, 15)
            except ValueError:
                self.report_id = None
        else:
            self.report_id = None

        self.usb_path = '%s:%s' % (
            usb_dev.bus, '.'.join('%u' % n for n in usb_dev.port_numbers))

    def __str__(self):
        return 'XTI(%s)' % self.serial_num

    def _set_configuration(self, bConfigurationValue):
        with self.lock:
            cfg = None
            try:
                cfg = self.usb_dev.get_active_configuration()
            except usb.core.USBError as e:
                if e.strerror != 'Configuration not set':
                    raise

            if cfg is None or cfg.bConfigurationValue != bConfigurationValue:
                usb.util.dispose_resources(self.usb_dev)
                self.usb_dev.set_configuration(bConfigurationValue)

    def _set_measurement_config(self):
        self._set_configuration(2)

    def read_measurement(self):
        '''
        Synchronously read a single measurement from the sensor, blocking if no
        measurement is currently available.
        '''
        with self.lock:
            p = self.usb_dev.read(0x81, 64)
        return Measurement._from_packet(self, p)

    def _yield_measurements(self, do_reset):
        with self.lock:
            if do_reset:
                self.usb_dev.reset()
            self._set_measurement_config()

            while not self._halt_yield:
                try:
                    yield self.read_measurement()
                except usb.core.USBError as e:
                    if e.errno != errno.ETIMEDOUT:
                        raise
                    continue

    def yield_measurements(self, do_reset=True):
        '''
        Yields Measurement objects synchronously in the current thread,
        blocking while waiting for new measurements to be acquired.
        '''
        with self.lock:
            self._halt_yield = False
            yield from self._yield_measurements(do_reset)

    def halt_yield(self):
        '''
        Halts an ongoing yield_measurements() call, causing it to eventually
        terminate the generator loop.
        '''
        self._halt_yield = True

    def _read_measurements_async(self, handler, do_reset):
        with self.lock:
            for m in self._yield_measurements(do_reset):
                handler(m)

    def read_measurements(self, handler, do_reset=True):
        '''
        Reads measurements asynchronously in a separate thread, calling the
        handler as measurements become available.  The handler should take a
        single Measurement object as an argument.
        '''
        with self.lock:
            assert self.thread is None
            self._halt_yield = False
            self.thread = threading.Thread(target=self._read_measurements_async,
                                           args=(handler, do_reset),
                                           daemon=False)
            self.thread.start()

    def join_read(self):
        '''
        Blocks the current thread until the asynchronous read thread completes.
        Typically this blocks indefinitely until some error occurs, however the
        read thread will also exit if someone sets the _halt_yield field to
        True (see XTI.halt_read()).
        '''
        self.thread.join()

    def halt_read(self):
        '''
        Halts any asynchronous measurement thread and waits for it to finish
        cleanly.
        '''
        self._halt_yield = True
        self.join_read()
