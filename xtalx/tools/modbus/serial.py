# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import time

import xtalx.tools.serial

from . import bus
from . import modbus_crc


class Bus(bus.Bus):
    '''
    Modbus is a trash protocol.  It's dumbfounding that a quarter of the way
    through the 21st century, people still want to use this decades-old insane
    protocol to try and talk to their devices.  But they do, so here we are.

    Modbus delimits requests and responses using a packet structure called a
    protocol data unit (PDU).  Modbus RTU, which is the main protocol used to
    talk to Modbus devices over a serial port, delimits the PDU using idle time
    on the serial bus.  For baud rates above 19200, an inter-character delay of
    over 750us is flagged as an error condition, and an idle time of 1.75ms is
    used to delimit the end of a PDU.

    Compliant Modbus target devices will watch for these gaps between
    characters and use them to delimit the start and end of PDUs being
    transmitted on the bus (either from some other target device or from the
    master device).  When responding to a request, a compliant Modbus device
    will ensure that the characters it transmits don't have any gaps between
    them and that the end of the PDU is marked by 1.75ms of silence.  For a
    target implementation on some MCU that is running on bare metal or with
    some sort of real-time or DMA facilities, transmitting the PDU without any
    gaps is generally easy to accomplish.

    On the master side, we may be running a Python script (such as this one)
    and using a generic USB-to-RS485 adapter to access the serial bus.  The
    host-side drivers will have buffers in the kernel and the adapter itself
    will probably also have buffers or FIFOs.  When the host transmits a PDU on
    the bus, as long as the PDU is fairly short (say, 32 bytes or less) then it
    has a pretty good chance of being transmitted without any gaps - it will
    fit inside a single USB packet and so will probably fit inside any FIFO on
    the adapter side and the hardware will then push it all out to the serial
    port as quickly as possible without any gaps.

    Things get hairy when we have to receive.  Instead of a chunk of data in a
    USB packet, the adapter is seeing characters trickle in on the serial bus.
    The adapter may choose to transmit some of those characters to the host
    right away, and then it may choose to buffer some of them for later.  Or,
    maybe the host polls the adapter once and then not again for awhile since
    it is busy servicing some other device on the USB bus (or busy doing
    something completely unrelated and doesn't have time for USB at all).  So,
    the host side sees a few bytes and then there is a gap - even though the
    target device transmitted things according to spec and the bytes are just
    sitting in a buffer somewhere waiting for someone to get to them.  If that
    gap is over 1.75ms in length, then a strictly-compliant host is going to
    interpret that as a PDU delimiter and try to process a truncated packet.

    All of this is pretty dumb.  Modbus is a master-slave request-response
    protocol.  The slave can't arbitrarily transmit and once a slave is
    addressed, no other device can transmit either.  We generally just use
    commands to read a fixed number of registers, so we know exactly how long
    of a response to expect (assuming no exception is returned from the device).
    There are some commands, such as the Read Identification command, which can
    return variable-length data, but never more than 252 bytes: including
    address, function code and 16-bit CRC, the maximum length of a Modbus RTU
    frame is 256 bytes.

    This Modbus host implementation drastically relaxes the rules when
    receiving data on the serial port.  We use a 100ms (default, but
    configurable) inter-character timeout as an end-of-frame delimiter for
    those variable-length frames.  For a frame where we know the response size
    ahead of time (such as Read Holding Registers) then we end the frame after
    exactly the expected number of bytes has been received, also detecting an
    exception frame and terminating the frame appropriately if one is received.
    '''
    def __init__(self, intf, baud_rate, parity='E', **kwargs):
        super().__init__()
        self.serial = xtalx.tools.serial.from_intf(intf, baudrate=baud_rate,
                                                   parity=parity, **kwargs)

    def _read_until_gap(self, prev_data):
        data = b''
        while True:
            byte = self.serial.read(1)
            if byte == b'':
                return data

            data += byte
            if len(data) > 256:
                raise bus.ResponseOverflowException(prev_data + data)

    def _process_response(self, slave_addr, function_code, nbytes, data):
        # Check the CRC.
        expected_crc = modbus_crc.compute_as_bytes(data[:-2])
        if expected_crc != data[-2:]:
            # We've received garbage.  In the case of a fixed-length response,
            # we haven't waited for an inter-frame gap yet, so do that now
            # allowing us to drain data from a babbling device.
            if nbytes is not None:
                data += self._read_until_gap(data)
            raise bus.BadCRCException(data, expected_crc)

        # The CRC was good, but we could still have logical issues with the
        # packet.  Make sure it is from the right address and has the right
        # function code.
        if data[0] != slave_addr:
            raise bus.BadAddressException(data)
        if data[1] & 0x7F != function_code:
            raise bus.BadFunctionException(data)
        if data[1] & 0x80:
            raise bus.ExceptionResponseException(data, data[2])

        # The response makes logical sense, we are golden.
        return data

    def _read_response(self, slave_addr, function_code, response_time_ms,
                       nbytes=None):
        '''
        Read the response from a target.  If nbytes is None, the response is
        variable-length and we use the inter-frame gap with an 0.1-second
        timeout to find it.  If nbytes is not None, it is the length of the
        response, not including the address byte or CRC byte, but including
        the function code and all data that follows.
        '''
        self.serial.timeout = response_time_ms / 1000
        data = b''

        # Read the address and function code bytes, bailing if there is a
        # timeout.
        data = self.serial.read(2)
        if len(data) < 2:
            raise bus.ResponseTimeoutException(data)

        # If we got an exception function code, switch to reading a fixed-
        # length exception response.
        if data[1] & 0x80:
            nbytes = 2

        # Read the remaining data.
        self.serial.timeout = 0.1
        if nbytes is not None:
            # We have the start of a fixed-length response that looks like what
            # we expect.  Read the remaining bytes.
            data += self.serial.read(nbytes + 1)
            if len(data) != nbytes + 3:
                raise bus.ResponseTimeoutException(data)
        else:
            # We have the start of a variable-length response that looks like
            # what we expect.  Read until we hit the inter-frame gap.
            data += self._read_until_gap(data)

        return self._process_response(slave_addr, function_code, nbytes, data)

    def _send_request(self, slave_addr, data):
        '''
        Send a request to the target slave address.  The request data starts at
        the function code field; we will insert the address at the start and
        append a valid CRC here.

        Before transmitting, we have to be sure that at least 1.75ms has
        elapsed since the most recent bus activity.  For instance, a compliant
        target device will ignore any request from us that starts immediately
        after it sent its most-recent response to us until it sees 1.75ms of
        idle time.  Since we are the only master, the easiest way to do that is
        to just always sleep here first.  We use a 2ms timeout just to be safe.
        '''
        time.sleep(0.002)
        data  = bytes([slave_addr]) + data
        data += modbus_crc.compute_as_bytes(data)
        self.serial.write(data)

    def transact(self, addr, data, response_time_ms, nbytes=None):
        self._send_request(addr, data)
        return self._read_response(addr, data[0], response_time_ms,
                                   nbytes=nbytes)
