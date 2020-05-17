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
    def __init__(self,map,vehicle_pose,previous_pose,disk_positions,vehicle_path,disk_paths,reached_goals,disk_being_pushed,pushed_disks,rrt,g):
        self._map = map
        self._vehicle_pose = vehicle_pose
        self._previous_pose = previous_pose
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


    def connectToPreviousPose(self,axis=False):
        if self._previous_pose == None:
            return True
        pq = queue.PriorityQueue()
        visitedNodes = {}
        previousPose = self._previous_pose
        currentPose = self._vehicle_pose
        starting_state = (currentPose.EuclideanDistance(previousPose),currentPose,[],0)
        pq.put(starting_state)
        while not pq.empty():
            curr_state = pq.get()
            (f,pose,path,g) = curr_state
            if pose == previousPose:
                path.append(pose)
                path.reverse()
                if axis!=False:
                    self.drawPath(path,axis)
                self._vehicle_path.append(path)
                return True
            if pose not in visitedNodes:
                visitedNodes[pose] = True
                new_path = path.copy()
                new_path.append(pose)
                for next_pose in self._RRT.tree[pose]:
                    if not self._RRT.edgeCollidesWithDirtPile(pose,next_pose,self._RRT.tree[pose][next_pose],self._disk_positions) and not next_pose in visitedNodes:
                        new_state = (next_pose.EuclideanDistance(previousPose),next_pose,new_path,g+self.getEdgeLength(pose,next_pose)) #change this to use the arc path length
                        pq.put(new_state)
                          

        return False


    def getEdgeLength(self,n1,n2):
        if n1 in self._RRT.tree:
            if n2 in self._RRT.tree[n1]:
                edge = self._RRT.tree[n1][n2]
                if isinstance(edge[0],bezier.curve.Curve):
                    return edge[0].length
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


    def drawPath(self,path,ax):
        for i in range(len(path)-1):
            curr_node = path[i]
            next_node = path[i+1]
            self._RRT.drawEdge(curr_node,next_node,ax,'k-')

        plt.draw()
        plt.pause(5)
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

    def getStateAfterPush(self,push_point,curr_disk_pos,disk_being_pushed,gValue):
        (closest_goal,found) = self.getClosestGoalToPushLine(curr_disk_pos)
        if not found:
            return False # do not push disk out of goal
        (new_disk_pos,new_vehicle_pose) = Pushing.pushDisk(push_point,curr_disk_pos,closest_goal,self._map)
        if not (curr_disk_pos[0] == new_disk_pos[0] and curr_disk_pos[1] == new_disk_pos[1]):
            distance = push_point.EuclideanDistance(new_vehicle_pose)
            new_edge = (distance,0,"F")
            inv_edge = (distance,0,"R")
            self._RRT.addEdge(new_vehicle_pose,push_point,new_edge,inv_edge)
            new_disk_positions = copy.deepcopy(self._disk_positions)
            new_disk_positions[disk_being_pushed] = new_disk_pos
            #vehicle_path.append(push_point)
            #vehicle_path.append(new_vehicle_pose)
            #new_vehicle_paths = copy.deepcopy(self._vehicle_path)
            #new_vehicle_paths.append(vehicle_path)
            new_disk_paths = copy.deepcopy(self._disk_paths)
            new_disk_paths[disk_being_pushed].append(curr_disk_pos)
            new_reached_goals = self.determineGoalsReached(new_disk_positions)
            new_pushed_disks = self._pushed_disks.copy()
            new_pushed_disks.append(disk_being_pushed)
            #use A* where g = distance between push points
            return PQState(self._map,new_vehicle_pose,self._vehicle_pose,new_disk_positions,self._vehicle_path,new_disk_paths,new_reached_goals,disk_being_pushed,new_pushed_disks,self._RRT,self._g+gValue)
           
        return False

    def getResultingStates(self,axis):
        resultingStates = []
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            push_point = self._vehicle_pose
            pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._disk_being_pushed,0)
            if pushedState != False:
                resultingStates.append(pushedState)
                print("Added pushing state from current pose")
           
            # next consider navigating to a different push point on the current disk
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius,self._vehicle_pose.theta)
            for push_point in new_push_points:
                if self._RRT.connectPushPoint(push_point):
                    pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._disk_being_pushed,BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y)))
                    if pushedState != False:
                        resultingStates.append(pushedState)
                        print("Added pushing state from new push point on same disk")
                       
                    
             
        # finally consider navigating to the push points of all other disks
        for i in range(len(self._disk_positions)):
            if i == self._disk_being_pushed:
                continue
            curr_disk_pos = self._disk_positions[i]
            new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius)
            for push_point in new_push_points:
                if self._RRT.connectPushPoint(push_point):
                    pushedState = self.getStateAfterPush(push_point,curr_disk_pos,i,BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y)))
                    if pushedState != False:
                        resultingStates.append(pushedState)
                        print("Added pushing state from push point on new disk")
                        
        return resultingStates


    def generatePosesAlongEdge(self,n1,n2,num_steps=20):
        poses = [n1]
        if n1 in self._RRT.tree:
            if n2 in self._RRT.tree[n1]:
                edge = self._RRT.tree[n1][n2]
                if edge != False: #this check should no longer be necessary
                    if isinstance(edge[0],bezier.curve.Curve):
                        (bezierEdge,direction) = edge
                        if direction == "F":
                            s = 0.0
                            while s<1.0:
                                point_list = bezierEdge.evaluate(s).tolist()
                                real_point = [point_list[0][0],point_list[1][0]]
                                next_list = bezierEdge.evaluate(s+0.025).tolist()
                                real_next = [next_list[0][0],next_list[1][0]]
                                newPose = Vehicle(real_point[0],real_point[1],math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(real_point,real_next))))
                                poses.append(newPose)
                                s+=0.025
                        else:
                            s=1.0-0.025
                            while s>0:
                                point_list = bezierEdge.evaluate(s).tolist()
                                real_point = [point_list[0][0],point_list[1][0]]
                                next_list = bezierEdge.evaluate(s+0.025).tolist()
                                real_next = [next_list[0][0],next_list[1][0]]
                                newPose = Vehicle(real_point[0],real_point[1],math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(real_point,real_next))))
                                poses.append(newPose)
                                s-=0.025
                    else:
                        (radius,theta,direction) = edge
                        if direction == "F" or direction == "R":
                            delta_distance = 0.025 #fix increment distance for straight lines
                            the_dist = delta_distance
                            while the_dist<radius:
                                new_position = n1.applyControl(the_dist,0,direction)
                                poses.append(new_position)
                                the_dist += delta_distance

                            new_position = n1.applyControl(radius,0,direction)
                            poses.append(new_position)
                        else:
                            delta_angle = theta/float(num_steps)
                            the_angle = delta_angle
                            for i in range(num_steps):
                                new_position = n1.applyControl(radius,the_angle,direction)
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
                elif j == len(curr_path)-2 and i==len(self._vehicle_path)-1:
                    a = 0
                    for final_pos in self._disk_positions:
                        self._disk_paths[a].append(final_pos)
                        a+=1

                    fig, ax = plt.subplots(1,1)
                    final_indices = [-1] * len(self._disk_positions)
                    ax = self._map.displayMap(ax,next_pose,self._disk_paths,final_indices)
                    fig.canvas.draw()       # draw the canvas, cache the renderer
                    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                    tp = fig.canvas.get_width_height()[::-1]
                    newtp = (tp[0]*2,tp[1]*2)
                    image  = image.reshape(newtp + (3,))
                    for b in range(375):
                        solution_images.append(image) #freeze on the final frame for 15 seconds
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
        
        plt.close("all")
        return solution_images