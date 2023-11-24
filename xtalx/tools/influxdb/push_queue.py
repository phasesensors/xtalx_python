# Copyright (c) 2020 by Phase Advanced Sensor Systems, Inc.
# All rights reserved.
import threading
import time
from influxdb import InfluxDBClient


class InfluxDBPushQueue:
    '''
    Class to asynchronously push data points to an InfluxDB instance.  Pushing
    points can take a nondeterministic length of time and by trying to push
    them synchronously you can introduce lots of jitter into your measurement.
    This asynchronous queue allows the work to be performed in a separate
    thread so as not to disturb the measurement times.
    '''
    def __init__(self, host, port, username, password, database=None,
                 push_cb=None, throttle_secs=0, **kwargs):
        self.push_cb = push_cb
        self.idb     = InfluxDBClient(host, port, username, password, **kwargs)
        if database:
            self.idb.switch_database(database)

        self.queue_cond    = threading.Condition()
        self.queue         = []
        self.cookie_queue  = []
        self.thread        = None
        self.running       = False
        self.throttle_secs = throttle_secs
        self.start()

    def start(self):
        assert not self.thread
        self.running = True
        self.thread  = threading.Thread(target=self._push_loop, daemon=True)
        self.thread.start()

    def append(self, p, cookie=None):
        '''
        Append a single point to the push queue.
        '''
        with self.queue_cond:
            self.queue.append(p)
            self.cookie_queue.append(cookie)
            self.queue_cond.notify()

    def append_list(self, ps, cookies=None):
        '''
        Append a list of points to the push queue.
        '''
        with self.queue_cond:
            self.queue += ps
            if cookies is not None:
                self.cookie_queue += cookies
            else:
                self.cookie_queue += [None] * len(ps)
            self.queue_cond.notify()

    def flush(self):
        '''
        Flush the current queue elements.
        '''
        while self.queue:
            pass
        with self.queue_cond:
            self.running = False
            self.queue_cond.notify()
        self.thread.join()
        self.thread = None
        self.start()

    def _push_loop(self):
        while self.queue or self.running:
            with self.queue_cond:
                while not self.queue and self.running:
                    self.queue_cond.wait()

                points            = self.queue
                cookies           = self.cookie_queue
                self.queue        = []
                self.cookie_queue = []

            if points:
                while points:
                    push_points = points[:10000]
                    points      = points[10000:]
                    while True:
                        try:
                            self.idb.write_points(push_points,
                                                  time_precision='n')
                            break
                        except Exception as e:
                            print('InfluxDB push exception: %s' % e)
                            print('Retrying in 30 seconds...')
                            time.sleep(30)

                    if self.push_cb:
                        for p, c in zip(push_points, cookies):
                            self.push_cb(p, c)

            time.sleep(self.throttle_secs)
