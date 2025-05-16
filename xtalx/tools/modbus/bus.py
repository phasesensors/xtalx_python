# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading


class ModbusException(Exception):
    pass


class ResponseException(ModbusException):
    def __init__(self, rx_packet, **kwargs):
        super().__init__(rx_packet.hex(), **kwargs)
        self.rx_packet = rx_packet


class ExceptionResponseException(ResponseException):
    def __init__(self, rx_packet, exception_number, **kwargs):
        super().__init__(rx_packet, **kwargs)
        self.exception_number = exception_number


class ResponseTimeoutException(ResponseException):
    pass


class BadCRCException(ResponseException):
    def __init__(self, rx_packet, expected_crc, **kwargs):
        super().__init__(rx_packet, **kwargs)
        self.expected_crc = expected_crc


class BadAddressException(ResponseException):
    pass


class BadFunctionException(ResponseException):
    pass


class BabbleException(ResponseException):
    # Target replied too quickly, before t3.5 expired.
    pass


class ResponseOverflowException(ResponseException):
    pass


class ResponseUnderflowException(ResponseException):
    pass


class ResponseInterruptedException(ResponseException):
    # We received some characters, then t1.5 expired, then we received more
    # characters.
    pass


class ResponseFramingErrorException(ResponseException):
    pass


class ResponseSyntaxException(ResponseException):
    pass


class DeviceIDObject:
    def __init__(self, object_id, value):
        self.object_id = object_id
        self.value     = value


class Bus:
    def __init__(self):
        self.lock = threading.Lock()

    def transact(self, addr, data, response_time_ms, nbytes=None):
        '''
        Perform a command/response transaction on the bus, initiated by sending
        the specified data bytes to the specified address.  The response_time_ms
        timeout is how long we are willing to wait for the first byte of the
        response.  This method returns the response data as a bytes() object.

        If response_time_ms is 0, then no response is expected (perhaps this was
        a broadcast write), we don't try to read one, and this method will
        return None instead.

        The nbytes parameter is an optional hint to the implementation on how
        many bytes to expect in a response.  It is useful for the serial
        implementation which can't meet the strict Modbus timing requirements
        to delimit PDUs.
        '''
        raise NotImplementedError

    def read_device_identification(self, slave_addr, read_code, object_id,
                                   response_time_ms=100):
        '''
        Performs a Read Device Identification request with the specified Read
        Device ID code and Object Id.

        Returns a list of DeviceIDObject objects, which have object_id and
        value fields corresponding to the same fields in the Modbus spec.
        '''
        with self.lock:
            rsp = self.transact(slave_addr, bytes([0x2B, 0x0E, read_code,
                                                   object_id]),
                                response_time_ms)

        nobjs  = rsp[7]
        objs   = []
        offset = 8
        for _ in range(nobjs):
            object_id = rsp[offset]
            size      = rsp[offset + 1]
            value     = rsp[offset + 2: offset + 2 + size]
            offset   += 2 + size
            objs.append(DeviceIDObject(object_id, value))

        return objs

    def read_holding_registers_binary(self, slave_addr, address, nregs,
                                      response_time_ms=100):
        '''
        Reads nregs 16-bit registers from the target device slave_addr starting
        with register address.  The return value is the raw binary data in the
        exact order it was streamed from the serial bus.  No splitting into
        16-bit values or byte-swapping of any sort is done; this returns the
        raw data.
        '''
        nbytes = 2 + nregs * 2
        with self.lock:
            rsp = self.transact(slave_addr, bytes([0x03,
                                                   (address >> 8) & 0xFF,
                                                   address & 0xFF,
                                                   (nregs >> 8) & 0xFF,
                                                   nregs & 0xFF]),
                                response_time_ms, nbytes=nbytes)
        if rsp[2] != 2 * nregs:
            raise ResponseSyntaxException(rsp)

        return rsp[3:-2]

    def write_holding_registers_binary(self, slave_addr, address, data,
                                       response_time_ms=100):
        '''
        Writes data to the target device slave_addr at register address.  The
        data is streamed out in order and no byte-swapping is performed.  Try
        to keep writes short so that there is no danger of a write being broken
        up in a kernel or adapter buffer.
        '''
        assert len(data) % 2 == 0
        assert 1 <= len(data) <= 123
        nregs = len(data) // 2
        with self.lock:
            rsp = self.transact(slave_addr, bytes([0x10,
                                                   (address >> 8) & 0xFF,
                                                   address & 0xFF,
                                                   (nregs >> 8) & 0xFF,
                                                   nregs & 0xFF,
                                                   len(data)]) + data,
                                response_time_ms, nbytes=5)
        if ((rsp[2] << 8) | rsp[3]) != address:
            raise ResponseSyntaxException(rsp)
        if ((rsp[4] << 8) | rsp[5]) != nregs:
            raise ResponseSyntaxException(rsp)
