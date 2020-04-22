import math
import queue
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

    def addEdge(self,node1,node2,push_point,tree):
        pass

    def PushToGoals(self,disk_num,disk_pos):
        pq = queue.PriorityQueue()
        firstState = PushState(0,disk_pos,[False]*len(self._map.goal_pos_xy),0)
        pq.put(firstState)
        currState = pq.get()
        while (False in currState.getStatuses()) and (not pq.empty()):
            if not currState.diskAtGoal():
                pushingActions = currState.getPushingActions()
                for action in pushingActions:
                    oldDiskPos = currState.getDiskPos()
                    newDiskPos = action[0]
                    push_point = action[1]
                    newStatuses = self.updateStatuses(currState.getStatuses(),newDiskPos)
                    self.graph[oldDiskPos][newDiskPos] = push_point
                    newState = PushState(currState.getG()+self.getHeursitic(newDiskPos,newStatuses),newDiskPos,newStatuses,currState.getG()+BasicGeometry.ptDist(oldDiskPos,newDiskPos))
                    pq.put(newState)





