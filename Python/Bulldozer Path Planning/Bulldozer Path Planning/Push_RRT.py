import math
import queue
import time
import turtle
import random
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
        self._max_push_distance = max_push_distance

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


    # Node in the push graph are point tuples representing the location of the centre of a disk
    # Edges are the push points used to move disk from n1 to n2
    def addEdge(self,node1,node2,push_point,axis=False):
        if axis!=False:
            axis.plot([node1[0],node2[0]],[node1[1],node2[1]],'m-',linewidth=1)
            plt.draw()
            plt.pause(0.01)
            plt.show(block=False)
            plt.pause(1)
        if node1 in self._graph:
            self._graph[node1][node2] = push_point
        else:
            self._graph[node1] = {}
            self._graph[node1][node2] = push_point

    def diskAtGoal(self,curr_disk_pos):
        for pos in self._map.goal_pos_xy:
            if self.diskAtThisGoal(curr_disk_pos,pos):
                return True
        return False


    def pushingCollision(self,push_point,disk_pos):
        edges = self._map.getMapEdgesAndObstacles()
        newLine = [[push_point.x,push_point.y],[disk_pos[0],disk_pos[1]]]
        for edge in edges:
            if BasicGeometry.doLinesIntersect(newLine,edge):
                return True
        return False

    def pushDisk(self,push_point,curr_disk_pos,num_steps=50):
        closestGoal = self.getClosestGoal(curr_disk_pos)
        bestPush = None
        min_dist = math.inf
        found = False
        if BasicGeometry.ptDist(closestGoal,curr_disk_pos) < self._max_push_distance:
            upperBound = BasicGeometry.ptDist(closestGoal,curr_disk_pos)
            lowerBound = 0.0
        else:
            upperBound = self._max_push_distance
            lowerBound = self._max_push_distance/2.0
        for i in range(num_steps):
            r = random.uniform(lowerBound,upperBound)
            new_disk_pos = (curr_disk_pos[0]+r*math.cos(math.radians(push_point.theta)),curr_disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
            dist = BasicGeometry.manhattanDistance(new_disk_pos,closestGoal)
            if dist < min_dist and not self.pushingCollision(push_point,new_disk_pos):
                min_dist = dist
                bestPush = new_disk_pos
                found = True

        if not found:
            return curr_disk_pos
        else:
            return bestPush

    def getClosestGoal(self,curr_disk_pos):
        min_dist = math.inf
        closestGoal = None
        found = False
        for pos in self._map.goal_pos_xy:
            dist = BasicGeometry.manhattanDistance(curr_disk_pos,pos)
            if dist < min_dist:
                closestGoal = pos
                min_dist = dist
                found = True
        
        if not found:
            raise Exception("Could not find closest goal")
        return closestGoal
        

    def getPushingActions(self,state,axis=False):
        push_points = self.getPushPoints(state.getDiskPos())
        pushing_actions = []
        for push_point in push_points:
            #print("Testing push point (%.2f.%.2f,%.2f)" % (push_point.x,push_point.y,push_point.theta))
            if self._RRT.connectPushPoint(push_point,axis):
                #print("Push point is accessible")
                #plt.draw()
                #plt.pause(0.01)
                #plt.show(block=False)
                #plt.pause(0.01)
                action = [self.pushDisk(push_point,state.getDiskPos()),push_point]
                pushing_actions.append(action)
                #create pushing action and add to list
            else:
                #print("Push point is not accessible")
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

    # Breath first search to the goal using a queue
    # Pushing action is directed towards goals using the manhattan distance heuristic
    def PushToGoals(self,disk_num,disk_pos,ax,ax2=False):
        disk_position = disk_pos.tolist()
        starting_pos = (disk_position[0],disk_position[1])
        q = queue.Queue()
        firstState = PushState(0,starting_pos,[False]*len(self._map.goal_pos_xy),0)
        q.put(firstState)
        while not q.empty():
            currState = q.get()
            if not (False in currState.getStatuses()):
                break
            if not self.diskAtGoal(currState.getDiskPos()): #upgrade this to not allow disks to be partially pushed in and out of goals
                pushingActions = self.getPushingActions(currState)
                for action in pushingActions:
                    oldDiskPos = currState.getDiskPos()
                    newDiskPos = action[0]
                    push_point = action[1]
                    if not self.samePosition(newDiskPos,oldDiskPos):
                        newStatuses = self.updateStatuses(currState.getStatuses(),newDiskPos)
                        self.addEdge(oldDiskPos,newDiskPos,push_point,ax2)
                        newState = PushState(currState.getG()+self.getHeuristic(newDiskPos,newStatuses),newDiskPos,newStatuses,currState.getG()+BasicGeometry.ptDist(oldDiskPos,newDiskPos))
                        q.put(newState)

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
                x_points = [node[0],n2[0]]
                y_points = [node[1],n2[1]]
                axis.plot(x_points,y_points,'m-',linewidth=1)



