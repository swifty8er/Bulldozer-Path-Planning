import math
import queue
import bezier
import copy
import numpy as np
from BasicGeometry import BasicGeometry
from Pushing import Pushing
from Vehicle import Vehicle
from TranspositionTable import TranspositionTable
from matplotlib import pyplot as plt


class PQState:
    def __init__(self,map,vehicle_pose,disk_positions,vehicle_path,disk_paths,reached_goals,disk_being_pushed,pushed_disks,rrt,g):
        self._map = map
        self._vehicle_pose = vehicle_pose
        self._disk_positions = disk_positions
        self._vehicle_path = vehicle_path
        self._disk_paths = disk_paths
        self._reached_goals = reached_goals
        self._pushed_disks = pushed_disks
        self._RRT = rrt
        self._disk_being_pushed = disk_being_pushed #index of disk being pushed -1 = no disk
        self._g = g
        self._f = self._g + self.calculateHeuristicValue()

    
    @property
    def f(self):
        return self._f

    @property
    def vehicle_pose(self):
        return self._vehicle_pose

    @property
    def disk_positions(self):
        return self._disk_positions

    def __lt__(self,other):
        return self.f < other.f


    def plotState(self,ax,line_width=2):
        self._map.plotMapBoundaryObstacles(ax)
        for disk_pos in self._disk_positions:
            disk_circle = BasicGeometry.circlePoints(disk_pos, self._map.disk_radius, 25)
            ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
        self.drawVehiclePose(ax)


    # a hacky zobrist hash - does not use random numbers
    def __hash__(self):
        h = 0
        h = h^self.vehicle_pose.__hash__()
        for pos in self.disk_positions:
            t = (pos[0],pos[1])
            h = h^hash(t)
        return h

    def isFinishState(self):
        for r in self._reached_goals:
            if not r:
                return False
        return True


    def calculateHeuristicValue(self):
        reached = self._reached_goals.copy()
        h = 0
        for disk in self._disk_positions:
            #check if disk in goal
            if not self.diskInGoal(disk):
                (closestGoal,found) = self.getClosestGoalManhattan(disk,reached)
                if found:
                    index = self._map.goal_pos_xy.index(closestGoal)
                    reached[index] = True
                    h += BasicGeometry.manhattanDistance(disk,closestGoal)
        return h


    def diskInGoal(self,disk_pos):
        for goal_pos in self._map.goal_pos_xy:
            if (BasicGeometry.ptDist(disk_pos,goal_pos)) < 0.05:
                return True
        return False

    def getClosestGoalManhattan(self,disk_pos,reached):
        shortestDist = math.inf
        closestGoal = None
        i = 0
        found = False
        for goal_pos in self._map.goal_pos_xy:
            if not reached[i]:
                dist = BasicGeometry.manhattanDistance(disk_pos,goal_pos)
                if dist < shortestDist:
                    shortestDist = dist
                    closestGoal = goal_pos
                    found = True
            i+=1

        
        return (closestGoal,found)

    def getClosestGoalToPushLine(self,curr_disk_position):
        shortestDist = math.inf
        closestGoal = None
        i = 0
        found = False
        for goal_pos in self._map.goal_pos_xy:
            if not self._reached_goals[i]:
                angle_between_heading_and_goal = abs(math.radians(self._vehicle_pose.theta) - BasicGeometry.vector_angle(BasicGeometry.vec_from_points(curr_disk_position,goal_pos)))
                dist = BasicGeometry.GoalDistanceMetric(curr_disk_position[0],curr_disk_position[1],angle_between_heading_and_goal,goal_pos[0],goal_pos[1])
                if dist < shortestDist:
                    shortestDist = dist
                    closestGoal = goal_pos
                    found = True

            i+=1

        return (closestGoal,found)


    # Use the RRT to find a path between the current position of the vehicle and the push_point
    # Use A* search in reverse from the push point to the current position
    def navigateToPushPoint(self,push_point,cachedPaths,axis):
        pq = queue.PriorityQueue()
        visitedNodes = {}
        starting_state = (push_point.EuclideanDistance(self._vehicle_pose),push_point,[],0)
        pq.put(starting_state)
        while not pq.empty():
            curr_state = pq.get()
            (f,pose,path,g) = curr_state
            if pose == self._vehicle_pose:
                path.append(pose)
                path.reverse()
                self.drawPath(path,axis)
                return (push_point,path,g)
            remainingPath = self.getSavedPath(pose,cachedPaths)
            if remainingPath != False:
                path.append(pose)
                path.reverse()
                new_path  = remainingPath + path
                self.drawPath(new_path,axis)
                return (push_point,new_path,g+self.calcPathLength(remainingPath))


            if pose not in visitedNodes:
                visitedNodes[pose] = True
                new_path = path.copy()
                new_path.append(pose)
                for next_pose in self._RRT.tree[pose]:
                    if not self._RRT.edgeCollidesWithDirtPile(pose,next_pose,self._RRT.tree[pose][next_pose],self._disk_positions) and not next_pose in visitedNodes:
                        new_state = (next_pose.EuclideanDistance(self._vehicle_pose),next_pose,new_path,g+self.getEdgeLength(pose,next_pose)) #change this to use the arc path length
                        pq.put(new_state)
                          

        return (False,False,False)


    def getEdgeLength(self,n1,n2):
        if n1 in self._RRT.tree:
            if n2 in self._RRT.tree[n1]:
                edge = self._RRT.tree[n1][n2]
                if isinstance(edge,bezier.curve.Curve):
                    return edge.length
                else:
                    try:
                        (radius,deltaTheta,direction) = edge
                    except:
                        raise Exception("Invalid RRT edge found")
                    if direction == "F" or direction =="R":
                        return radius
                    else:
                        return radius * math.radians(deltaTheta)
            else:
                print("Edge for length not found")
                return 0
        else:
            print("Edge for length not found")
            return 0


    def calcPathLength(self,path):
        length = 0
        for i in range(len(path)-1):
            curr_node = path[i]
            next_node = path[i+1]
            length += self.getEdgeLength(curr_node,next_node)
        return length

    def getSavedPath(self,pose,cachedPaths):
        for path in cachedPaths:
            for i in range(len(path)):
                curr_pose = path[i]
                if curr_pose == pose:
                    return path[:i]
        return False

    def drawPath(self,path,ax):
        for i in range(len(path)-1):
            curr_node = path[i]
            next_node = path[i+1]
            self._RRT.drawEdge(curr_node,next_node,ax,'k-')

        plt.draw()
        plt.pause(1)
        plt.show(block=False)

    def drawVehiclePose(self,axis):
        vehicle_pos = (self.vehicle_pose.x,self.vehicle_pose.y)
        pos_circle = BasicGeometry.circlePoints(vehicle_pos, self._map.disk_radius, 25)
        axis.plot(pos_circle[0],pos_circle[1],color='red', linewidth=2)
        r = 0.5
        dx = r*math.cos(math.radians(self.vehicle_pose.theta))
        dy = r*math.sin(math.radians(self.vehicle_pose.theta))
        axis.arrow(self.vehicle_pose.x,self.vehicle_pose.y,dx,dy,width=0.05)
        plt.draw()
        plt.pause(1)
        plt.show(block=False)


    def determineGoalsReached(self,disk_positions):
        reached = [False]*len(self._map.goal_pos_xy)
        i = 0
        for goal_pos in self._map.goal_pos_xy:
            for disk_pos in disk_positions:
                if (BasicGeometry.ptDist(disk_pos,goal_pos)) < 0.05:
                    reached[i] = True
                    break
            i+=1
        return reached

    def getStateAfterPush(self,push_point,curr_disk_pos,vehicle_path,disk_being_pushed,gValue):
        (closest_goal,found) = self.getClosestGoalToPushLine(curr_disk_pos)
        if not found:
            return False # do not push disk out of goal
        (new_disk_pos,new_vehicle_pose) = Pushing.pushDisk(push_point,curr_disk_pos,closest_goal,self._map)
        if not (curr_disk_pos[0] == new_disk_pos[0] and curr_disk_pos[1] == new_disk_pos[1]):
            distance = push_point.EuclideanDistance(new_vehicle_pose)
            new_edge = (distance,0,"F")
            inv_edge = (distance,0,"R")
            self._RRT.addEdge(new_vehicle_pose,push_point,new_edge,inv_edge)
            new_disk_positions = np.copy(self._disk_positions)
            new_disk_positions[disk_being_pushed] = new_disk_pos
            vehicle_path.append(push_point)
            vehicle_path.append(new_vehicle_pose)
            new_vehicle_paths = copy.deepcopy(self._vehicle_path)
            new_vehicle_paths.append(vehicle_path)
            new_disk_paths = copy.deepcopy(self._disk_paths)
            new_disk_paths[disk_being_pushed].append(curr_disk_pos)
            new_reached_goals = self.determineGoalsReached(new_disk_positions)
            new_pushed_disks = self._pushed_disks.copy()
            new_pushed_disks.append(disk_being_pushed)
            #use greedy search for now i.e g = 0 f = h
            return PQState(self._map,new_vehicle_pose,new_disk_positions,new_vehicle_paths,new_disk_paths,new_reached_goals,disk_being_pushed,new_pushed_disks,self._RRT,0)
           
        return False

    def getResultingStates(self,axis):
        resultingStates = []
        cachedPaths = []
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            push_point = self._vehicle_pose
            pushedState = self.getStateAfterPush(push_point,curr_disk_pos,[],self._disk_being_pushed,self._g)
            if pushedState != False:
                resultingStates.append(pushedState)
           
            # next consider navigating to a different push point on the current disk
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius,self._vehicle_pose.theta)
            for push_point in new_push_points:
                if self._RRT.connectPushPoint(push_point):
                    (new_vehicle_pose,new_vehicle_path,gValue) = self.navigateToPushPoint(push_point,cachedPaths,axis)
                    if not (new_vehicle_pose == False and new_vehicle_path == False and gValue == False):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,new_vehicle_path,self._disk_being_pushed,self._g+gValue)
                        if pushedState != False:
                            resultingStates.append(pushedState)
                       
                        cachedPaths.append(new_vehicle_path)
                    
             
        # finally consider navigating to the push points of all other disks
        for i in range(len(self._disk_positions)):
            if i == self._disk_being_pushed:
                continue
            curr_disk_pos = self._disk_positions[i]
            new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius)
            for push_point in new_push_points:
                if self._RRT.connectPushPoint(push_point):
                    (new_vehicle_pose,new_vehicle_path,gValue) = self.navigateToPushPoint(push_point,cachedPaths,axis)
                    if not (new_vehicle_pose == False and new_vehicle_path == False and gValue == False):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,new_vehicle_path,i,self._g+gValue)
                        if pushedState != False:
                            resultingStates.append(pushedState)
                        
                        cachedPaths.append(new_vehicle_path)
        return resultingStates


    def generatePosesAlongEdge(self,n1,n2,edge,num_steps=20):
        poses = [n1]
        if n1 in self._RRT.tree:
            if n2 in self._RRT.tree[n1]:
                edge = self._RRT.tree[n1][n2]
                if isinstance(edge,bezier.curve.Curve):
                    s = 0.0
                    while s<1.0:
                        point = edge.evaluate(s)
                        next_point = edge.evaluate(s+0.01)
                        newPose = Vehicle(point[0],point[1],math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(point,next_point))))
                        poses.append(newPose)
                        s+=0.01
                else:
                    angle = edge[1]
                    delta_angle = angle/float(num_steps)
                    the_angle = delta_angle
                    for i in range(num_steps):
                        new_position = start_position.applyControl(edge[0],the_angle,edge[2])
                        poses.append(new_position)
                        the_angle += delta_angle
        poses.append(n2)
        return poses

    def plotSolution(self):

        solution_images = []
        #all_nodes = self._pg.nodes + self._pg.push_points + self._pg.dest_points
        #curr_disk_index = [0]*len(disks_path)
        #curr_disk_pos = []
        #push_point_rng = [self.num_of_nodes, self.num_of_nodes + int(self.num_of_points/2) - 1]
        #dest_points_rng = [self.num_of_nodes + int(self.num_of_points/2), self.total_num_nodes - 1]
        #for curr_disk_path in disks_path:
        #    curr_disk_pos.append(all_nodes[curr_disk_path[0]])
        disk_pos_indices = [0]*len(self._disk_paths)
        for i in range(len(self._vehicle_path)):
            #get and plot vehicle position
            curr_path = self._vehicle_path[i]
            for j in range(len(curr_path)-1):
                curr_pose = curr_path[j]
                next_pose = curr_path[j+1]
                if j == len(curr_path)-2 and i<len(self._vehicle_path)-1:
                    #push the disk
                    disk_being_pushed = self._pushed_disks[i]
                    disk_pos_indices[disk_being_pushed] +=1
                    
                    fig, ax = plt.subplots(1,1)
                   
                    ax = self._map.displayMap(ax,next_pose,self._disk_paths,disk_pos_indices)
                    fig.canvas.draw()       # draw the canvas, cache the renderer
                    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                    tp = fig.canvas.get_width_height()[::-1]
                    newtp = (tp[0]*2,tp[1]*2)
                    image  = image.reshape(newtp + (3,))
                    solution_images.append(image)
                    plt.close(fig)
                else:
                  
                    edge_path = self.generatePosesAlongEdge(curr_pose,next_pose)
                    for k in range(len(edge_path)):

                        fig, ax = plt.subplots(1,1)
                   
                        ax = self._map.displayMap(ax,edge_path[k],self._disk_paths,disk_pos_indices)
                        fig.canvas.draw()       # draw the canvas, cache the renderer
                        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                        tp = fig.canvas.get_width_height()[::-1]
                        newtp = (tp[0]*2,tp[1]*2)
                        image  = image.reshape(newtp + (3,))
                        solution_images.append(image)
                        plt.close(fig)

        #add the final disk positions to the state disk paths
        a = 0
        for final_pos in self._disk_positions:
            self._disk_paths[a].append(final_pos)
            a+=1

        fig, ax = plt.subplots(1,1)
        final_indices = [-1] * len(self._disk_positions)
        ax = self._map.displayMap(ax,self._vehicle_pose,self._disk_paths,final_indices)
        fig.canvas.draw()       # draw the canvas, cache the renderer
        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
        tp = fig.canvas.get_width_height()[::-1]
        newtp = (tp[0]*2,tp[1]*2)
        image  = image.reshape(newtp + (3,))
        solution_images.append(image)
        plt.close(fig)
        plt.close("all")
        return solution_images