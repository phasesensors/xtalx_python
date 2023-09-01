# Copyright (c) 2020-2023 Phase Advanced Sensor Systems, Inc.
# All rights reserved.


class InvalConfigException(Exception):
    pass


class Config:
    def __init__(self, lines, required_keys):
        self.kv_pairs = {}
        rks = set(required_keys)
        for l in lines:
            if l.startswith('#'):
                continue
            k, v = l.split()
            if k in self.kv_pairs:
                raise InvalConfigException('Key %s repeated.' % k)
            self.kv_pairs[k] = v
            rks.discard(k)
        if rks:
            raise InvalConfigException('Missing keys: %s' % rks)

    def has_keys(self, required_keys):
        for k in required_keys:
            if k not in self.kv_pairs:
                return False
        return True

    def __getattr__(self, k):
        return self.kv_pairs[k]
