from BasicGeometry import BasicGeometry
import math
import random
class PushState:
    def __init__(self,f,disk_pos,statuses,g):
        self._f = f
        self._disk_pos = disk_pos
        self._statuses = statuses
        self._g = g

    def __lt__(self,other):
        return self.f < other.f


    # disk_pos is a point tuple (x,y)
    def getDiskPos(self):
        return self._disk_pos

    def getStatuses(self):
        return self._statuses

    def getG(self):
        return self._g

    @property
    def f(self):
        return self._f



   


