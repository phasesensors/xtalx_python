# Copyright (c) 2024 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.


def crc16_byte_reversed(v, P):
    for _ in range(8):
        v = (v >> 1) ^ (P * (v & 1))
    return v & 0xFFFF


LUT = [crc16_byte_reversed(i, 0xA001) for i in range(256)]


def compute(data, v=0xFFFF):
    for d in data:
        v = (v >> 8) ^ LUT[(v ^ d) & 0xFF]
    return v


def compute_as_bytes(data, **kwargs):
    crc = compute(data, **kwargs)
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


assert compute(bytes([0x01, 0x04, 0x08, 0x00, 0x00, 0x00, 0x09, 0x00,
                      0x00, 0x00, 0x00])) == 0x0CF8
