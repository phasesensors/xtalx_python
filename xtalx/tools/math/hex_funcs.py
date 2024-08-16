# Copyright (c) 2022-2024 by Phase Advanced Sensor Systems, Inc.
import struct


def hex_to_double(v):
    '''
    Converts a 64-bit integer representation of a double-precision floating
    point number into its actual floating-point value.  I.e., the Python
    equivalent of this C union:

        union
        {
            uint64_t    u;
            double      d;
        };
    '''
    return struct.unpack('<d', struct.pack('<Q', v))[0]


def hex_s_to_double(v):
    '''
    Converts a 64-bit value encoded as a hex string into a floating-point
    double value.
    '''
    return hex_to_double(int(v, 16))
