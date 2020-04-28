import math
import queue
import time
import turtle
import matplotlib.pyplot as plt
from BasicGeometry import BasicGeometry
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle
from PushState import PushState
import matplotlib.pyplot as plt
SCALING = 100.0
OFFSET = 300.0

class Push_RRT:
    def __init__(self,map,max_push_distance):
        self._map = map
        self._graph = {}
        self._RRT = None
        self._map_push_distance = max_push_distance

    def setRRT(self,rrt):
        self._RRT = rrt

    def getPushPoints(self,disk_pos):
        push_points = []
        angle = 0
        while (angle<=2*math.pi):
            push_point = (disk_pos[0]+self._map.disk_radius*math.cos(angle),disk_pos[1]+self._map.disk_radius*math.sin(angle))
            heading = math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(push_point,disk_pos)))
            push_pose = Vehicle(push_point[0],push_point[1],heading%360)
            push_points.append(push_pose)
            angle += (math.pi/6.0)
        return push_points

    def addEdge(self,node1,node2,push_point):
        
        if node1 in self._graph:
            self._graph[node1][node2] = push_point
        else:
            self._graph[node1] = {}
            self._graph[node1][node2] = push_point


    def getPushingActions(self,state,axis=False):
        push_points = self.getPushPoints(state.getDiskPos())
        pushing_actions = []
        for push_point in push_points:
            print("Testing push point (%.2f.%.2f,%.2f)" % (push_point.x,push_point.y,push_point.theta))
            if self._RRT.connectPushPoint(push_point,axis):
                print("Push point is accessible")
                #plt.draw()
                #plt.pause(0.01)
                #plt.show(block=False)
                #plt.pause(0.01)
                action = [state.pushDisk(push_point,self._map.goal_pos_xy,self._map_push_distance,50),push_point]
                pushing_actions.append(action)
                #create pushing action and add to list
            else:
                print("Push point is not accessible")
                action = [state.getDiskPos(),push_point]
                pushing_actions.append(action)
        #plt.draw()
        #plt.pause(1)
        #plt.show(block=False)
        return pushing_actions


    def getHeuristic(self,newPosition,newStatuses):
        h = 0
        i = 0
        for status in newStatuses:
            if not status:
                h += BasicGeometry.manhattanDistance(newPosition,self._map.goal_pos_xy[i])
            i+=1
        return h

    def updateStatuses(self,statuses,disk_pos):
        newStatuses = []
        for i in range(len(self._map.goal_pos_xy)):
            goal_pos = self._map.goal_pos_xy[i]
            if statuses[i]:
                newStatuses.append(True)
            else:
                if self.diskAtThisGoal(disk_pos,goal_pos):
                    newStatuses.append(True)
                else:
                    newStatuses.append(False)
        return newStatuses


    def diskAtThisGoal(self,disk_pos,goal_pos):
        if abs(disk_pos[0]-goal_pos[0]) < 0.05 and abs(disk_pos[1]-goal_pos[1]) < 0.05:
            return True
        else:
            return False

    def PushToGoals(self,disk_num,disk_pos,ax):
        disk_position = disk_pos.tolist()
        starting_pos = (disk_position[0],disk_position[1])
        pq = queue.PriorityQueue()
        firstState = PushState(0,starting_pos,[False]*len(self._map.goal_pos_xy),0)
        pq.put(firstState)
        while not pq.empty():
            currState = pq.get()
            if not (False in currState.getStatuses()):
                break
            if not currState.diskAtGoal(self._map.goal_pos_xy):
                pushingActions = self.getPushingActions(currState)
                for action in pushingActions:
                    oldDiskPos = currState.getDiskPos()
                    newDiskPos = action[0]
                    push_point = action[1]
                    if not self.samePosition(newDiskPos,oldDiskPos):
                        newStatuses = self.updateStatuses(currState.getStatuses(),newDiskPos)
                        self.addEdge(oldDiskPos,newDiskPos,push_point)
                        newState = PushState(currState.getG()+self.getHeursitic(newDiskPos,newStatuses),newDiskPos,newStatuses,currState.getG()+BasicGeometry.ptDist(oldDiskPos,newDiskPos))
                        pq.put(newState)

    def samePosition(self,pos1,pos2):
        (x1,y1) = pos1
        (x2,y2) = pos2
        if (x1==x2) and (y1==y2):
            return True
        else:
            return False

    def draw(self,axis):
        for node in self._graph:
            for n2 in self._graph[node]:
                x_points = [node.x,n2.x]
                y_points = [node.y,n2.y]
                axis.plot(x_points,y_points,'m-',linewidth=1)



