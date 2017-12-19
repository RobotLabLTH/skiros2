import time

"""
Usage
	With Timekeeper:
		--Do something--
	time_to_do_something = Timekeeper.list[-1]
	avg_time_to_do_something = Timekeeper.getAvgTime()
"""
class TimeKeeper():
    def __init__(self):
        self._list = []
        
    def __enter__(self):
        self._start_time = time.time()
        return self
        
    def __exit__(self, type, value, traceback):
        self._list.append(time.time()-self._start_time)

    def getAvgTime(self):
        return sum(self._list) / float(len(self._list))

    def getLast(self):
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
        if not self._map.has_key(key):
            self._map[key] = TimeKeeper()
        return self._map[key]
        
    def printAvgTimings(self):
        for k, keeper in self._map.iteritems():
            time = keeper.getAvgTime()
            print "Average {}: {}. {} times. ".format(k, time, len(keeper._list))
