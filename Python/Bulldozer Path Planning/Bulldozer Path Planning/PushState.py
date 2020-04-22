class PushState:
    def __init__(self,f,disk_pos,statuses,g):
        self._f = f
        self._disk_pos = disk_pos
        self._statuses = statuses
        self._g = g

    def getDiskPos(self):
        return self._disk_pos

    def getStatuses(self):
        return self._statuses

    def getG(self):
        return self._g


