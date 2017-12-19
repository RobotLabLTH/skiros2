class IdGen:
    def __init__(self):
        self._id = 0
        self._ids = []

    def getId(self, desired=-1):
        if desired >= 0:
            self._id = desired
        while self._id in self._ids:
            self._id += 1
        self._ids.append(self._id)
        return self._id

    def hasId(self, uid):
        return uid in self._ids

    def removeId(self, uid):
        self._ids.remove(uid)
        self._id = uid

    def clear(self):
        self._ids = []
        self._id = 0
