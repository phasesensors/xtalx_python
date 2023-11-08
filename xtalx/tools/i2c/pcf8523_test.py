import time

import xtalx.i2c_bridge


I2C_FREQ       = 10000
POLL_DELAY_SEC = 1


dev = xtalx.i2c_bridge.find_one()
ib  = xtalx.i2c_bridge.make(dev)
ib.busses[0].enable_with_pullups(I2C_FREQ)
while True:
    data = ib.busses[0].read(0x68, 20)
    print(data.hex())
    time.sleep(POLL_DELAY_SEC)
