# Copyright (c) 2025 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
from .xhtiss_091 import XHTISS_091
from .xhtiss_092 import XHTISS_092, crc8


def make_xhtiss(bus):
    # Probe the target firmware.
    tx_cmd = b'\x34\x00\x00'
    tx_cmd = tx_cmd + bytes([crc8(tx_cmd)])
    while True:
        # Send a NOP command.  This will be interpreted as an unsupported
        # command by 0.9.1 firmware.
        print('STX: %s' % tx_cmd.hex())
        rsp = bus.transact(tx_cmd)
        print('SRX: %s' % rsp.hex())
        if rsp == b'\xAA\xBB??':
            return XHTISS_091(bus)

        # The response didn't look like 0.9.1 firmware, analyze it.
        if rsp[0] != 0xAA:
            continue
        if rsp[1] != 0x00:
            continue
        if rsp[2] != 0x34:
            continue
        if rsp[3] != crc8(rsp[0:3]):
            continue

        # The response was good for an 0.9.2 or higher firmware; we can double-
        # check the sticky status code.
        cmd = b'\x00\x00'
        print('STX: %s' % cmd.hex())
        rsp = bus.transact(b'\x00\x00')
        print('SRX: %s' % rsp.hex())
        if rsp[0] != 0xAA:
            continue
        if rsp[1] != 0x00:
            continue

        return XHTISS_092(bus)
