from functools import wraps
from timeit import default_timer as now

import time
from . import logger as log


class Timer(object):
    def __init__(self, name):
        self._name = name
        self._start_time = now()
        self._time = self._start_time

    def reset(self):
        self._start_time = now()
        self._time = self._start_time

    def tic(self, desc=None):
        dt = now() - self._time
        log.info(self._name, "{} took {:0.4f} secs.".format(desc, dt))

    def toc(self, desc=None):
        self.tic(desc)
        self._time = now()

    def __enter__(self):
        self._start_time = now()
        self._time = self._start_time
        return self

    def __exit__(self, type, value, traceback):
        if type is not None:
            return False
        dt = now() - self._start_time
        log.info(self._name, "Total: {:0.4f} secs.".format(dt))
        return True


"""
Usage
	With Timekeeper:
		--Do something--
	time_to_do_something = Timekeeper.getLast()
	avg_time_to_do_something = Timekeeper.getAvgTime()
"""


class TimeKeeper():
    def __init__(self):
        self.reset()

    def reset(self):
        self._start_time = now()
        self._time = self._start_time
        self._list = []

    def time_from_start(self):
        return now() - self._start_time

    def tic(self):
        return now() - self._time

    def toc(self):
        dt = self.tic()
        self._time = now()
        return dt

    def __enter__(self):
        self._start_time = time.time()
        return self

    def __exit__(self, type, value, traceback):
        if len(self._list) > 10:
            self._list.pop(0)
        self._list.append(time.time() - self._start_time)

    def get_avg_time(self):
        if not self._list:
            return None
        return sum(self._list) / float(len(self._list))

    def get_last(self):
        if not self._list:
            return None
        return self._list[-1]


"""
Usage
	With Timekeepers[name]:
		--Do something--
	time_to_do_something = Timekeeper.list[-1]
"""


class TimeKeepers():
    def __init__(self):
        self._map = {}

    def __getitem__(self, key):
        if key not in self._map:
            self._map[key] = TimeKeeper()
        return self._map[key]

    def printAvgTimings(self):
        for k, keeper in self._map.items():
            time = keeper.getAvgTime()
            print("Average {}: {}. {} times. ".format(k, time, len(keeper._list)))
