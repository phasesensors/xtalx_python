# Copyright (c) 2023 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.

class Bus:
    def enable(self, freq):
        raise NotImplementedError

    def enable_with_pullups(self, freq):
        raise NotImplementedError

    def disable(self):
        raise NotImplementedError

    def read(self, addr7, nbytes):
        raise NotImplementedError

    def write(self, addr7, data):
        raise NotImplementedError
