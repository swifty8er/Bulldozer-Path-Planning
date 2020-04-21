import math
from BasicGeometry import BasicGeometry
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle

class Push_RRT:
    def __init__(self,map):
        self._map = map
        self._graph = {}

    def getPushPoints(self,disk_pos):
        push_points = []
        angle = 0
        while (angle<=2*math.pi):
            push_point = (disk_pos[0]+self._map.disk_radius*math.cos(angle),disk_pos[1]+self._map.disk_radius*math.sin(angle))
            heading = math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(push_point,disk_pos)))
            push_pose = Vehicle(push_point[0],push_point[1],heading)
            push_points.append(push_pose)
            angle += (math.pi/6.0)
        return push_points

    def addEdge(node1,node2,push_point,tree):
        pass




