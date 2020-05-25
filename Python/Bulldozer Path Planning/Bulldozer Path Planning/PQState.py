import math
import queue
import bezier
import copy
import numpy as np
import time
from BasicGeometry import BasicGeometry
from BezierLib import BezierLib
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
                (closestGoal,found) = self.getClosestGoalEuclidean(disk,reached)
                if found:
                    index = self._map.goal_pos_xy.index(closestGoal)
                    reached[index] = True
                    h += BasicGeometry.ptDist(disk,closestGoal)
        return h


    def diskInGoal(self,disk_pos):
        for goal_pos in self._map.goal_pos_xy:
            if (BasicGeometry.ptDist(disk_pos,goal_pos)) < 0.075:
                return True
        return False

    def getClosestGoalEuclidean(self,disk_pos,reached):
        shortestDist = math.inf
        closestGoal = None
        i = 0
        found = False
        for goal_pos in self._map.goal_pos_xy:
            if not reached[i]:
                dist = BasicGeometry.ptDist(disk_pos,goal_pos)
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

    def rollBackDiskPush(self):
        curr_disk_positions = []
        for i in range(len(self._disk_positions)):
            if i == self._disk_being_pushed:
                curr_disk_positions.append(self._disk_paths[i][-1])
            else:
                curr_disk_positions.append(self._disk_positions[i])
        return curr_disk_positions


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
                self._vehicle_path = copy.deepcopy(self._vehicle_path)
                self._vehicle_path.append(path)
                return True
            if len(path)>0:
                curr_disk_positions = self.rollBackDiskPush()
            else:
                curr_disk_positions = self._disk_positions
            if pose not in visitedNodes:
                visitedNodes[pose] = True
                new_path = path.copy()
                new_path.append(pose)
                for next_pose in self._RRT.tree[pose]:
                    if not self._RRT.edgeCollidesWithDirtPile(pose,next_pose,self._RRT.tree[pose][next_pose],curr_disk_positions) and not next_pose in visitedNodes:
                        new_g = self.getEdgeLength(pose,next_pose)
                        new_state = (g+new_g+next_pose.EuclideanDistance(previousPose),next_pose,new_path,g+new_g) #change this to use the arc path length
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
        plt.pause(0.01)
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
                if (BasicGeometry.ptDist(disk_pos,goal_pos)) < 0.075:
                    reached[i] = True
                    break
            i+=1
        return reached

    def getProjectionHeuristic(self,curr_pose,dest_pose):
        line = [math.cos(math.radians(dest_pose.theta)),math.sin(math.radians(dest_pose.theta))]
        x_vec = BasicGeometry.vec_from_points([dest_pose.x,dest_pose.y],[curr_pose.x,curr_pose.y])
        c = np.dot(line,x_vec)/np.dot(line,line)
        proj_vec = np.array(line)
        proj_vec = proj_vec * c
        mag = np.linalg.norm(proj_vec)
        if mag == 0:
            inv_mag = 1000000
        else:
            inv_mag = 1.0/mag

        print("Inside projection heuristic between curr_pose = (%.2f,%.2f,%.2f) and dest_pose = (%.2f,%.2f,%.2f)" %(curr_pose.x,curr_pose.y,curr_pose.theta,dest_pose.x,dest_pose.y,dest_pose.theta))
        print("Inverse magnitude = ",inv_mag)
        angular_similarity = ((1.1-math.cos(math.radians(abs(dest_pose.theta-curr_pose.theta))))*5)
        print("Angular similiarity = ",angular_similarity)
        total = inv_mag * angular_similarity
        print("Total h value = ",total)
        time.sleep(5)
        return total
    def calcEdgeLength(self,edge):
        try:
            (radius,deltaTheta,direction) = edge
            if direction == "F" or direction == "R":
                return radius
            else:
                return radius * math.radians(deltaTheta)
        except:
            raise Exception("Invalid edge passed to calc edge length function")


    def searchForBezierConnectablePoint(self,pose,dest_pose,curr_disk_positions,path,limit):
        print("Searching for bezier connectable point start pose = (%.2f,%.2f,%.2f) dest pose = (%.2f,%.2f,%.2f)" % (pose.x,pose.y,pose.theta,dest_pose.x,dest_pose.y,dest_pose.theta))
        pq = queue.PriorityQueue()
        starting_state = (self.getProjectionHeuristic(pose,dest_pose),pose,[],0)
        pq.put(starting_state)
        bestH = math.inf
        bestPose = None
        bestPath = []
        visitedPoses = {}
        i = 0
        while not pq.empty() and i < limit:
            curr_state = pq.get()
            (f,curr_pose,new_path,g) = curr_state
            curr_h = self.getProjectionHeuristic(curr_pose,dest_pose)
            print("Current pose in a star is (%.2f,%.2f,%.2f)" % (curr_pose.x,curr_pose.y,curr_pose.theta))
            if curr_h < 0.3:
                new_path.append(curr_pose)
                return (curr_pose,path+new_path)
            if curr_h < bestH:
                bestH = curr_h
                bestPose = curr_pose
                best_path = new_path

            if curr_pose not in visitedPoses:
                visitedPoses[curr_pose] = True
                new_path = path.copy()
                new_path.append(curr_pose)
                for (next_pose,edge) in curr_pose.getNextPoses():
                    print("Next pose is (%.2f,%.2f,%.2f)" % (next_pose.x,next_pose.y,next_pose.theta))
                    time.sleep(2)
                    if not self._RRT.isCollision(curr_pose,edge) and not self._RRT.nodeWithinRadiusOfDirtPile(next_pose,curr_disk_positions):
                        self._RRT.addEdge(next_pose,curr_pose,edge,self._RRT.getInverseControl(edge))
                        new_g = g + self.calcEdgeLength(edge)
                        next_state = (new_g+self.getProjectionHeuristic(next_pose,dest_pose),next_pose,new_path,new_g)
                        pq.put(next_state)
            i+=1
        if bestPose == None:
            raise Exception("A star search for bezier connection point failed")
        return (bestPose,path+best_path)

    def makeBezierConnectionToPreviousPose(self,ax=False):
        if self._previous_pose == None:
            return True
        path = [self._vehicle_pose]
        edges_to = list(self._RRT.tree[self._vehicle_pose].keys())
        #print("There are edges from the current vehicle pose (%.2f,%.2f,%.2f) to" % (self._vehicle_pose.x,self._vehicle_pose.y,self._vehicle_pose.theta))
        next_pose = edges_to[0]
        path.append(next_pose)
        if next_pose == self._previous_pose:
            self._vehicle_path = copy.deepcopy(self._vehicle_path)
            path.reverse()
            self._vehicle_path.append(path)
            return True
        curr_disk_positions = self.rollBackDiskPush()
        (final_pose,final_path) = self.searchForBezierConnectablePoint(self._previous_pose,next_pose,curr_disk_positions,path,100)
        print("Reverse from post push pose (%.2f,%.2f,%.2f)" % (next_pose.x,next_pose.y,next_pose.theta))
        print("Starting vehicle pose (prev pose) (%.2f,%.2f,%.2f)" % (self._previous_pose.x,self._previous_pose.y,self._previous_pose.theta))
        print("Better staring pose (after reversing a star search) (%.2f,%.2f,%.2f)" % (final_pose.x,final_pose.y,final_pose.theta))
        time.sleep(10)
        degree = 3
        iterations = 500
        while degree < 12:
            print("Finding bezier curve with degree %d and iterations %d" % (degree,iterations))
            bestCurve = BezierLib.getBestBezierCurveConnectionBetweenTwoPoses(final_pose,next_pose,self._map,curr_disk_positions,degree,iterations,50)
            if bestCurve != False:
                print("Found bezier connection")
                self._RRT.addEdge(final_pose,self._previous_pose,(bestCurve,"F"),False)
                if ax!=False:
                   bestCurve.plot(100,'red',ax=ax)
                self._vehicle_path = copy.deepcopy(self._vehicle_path)
                path.reverse()
                self._vehicle_path.append(final_path+path)
                return True
            degree += 1
            iterations *= 1.5
        return False



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
            new_g = self._g + gValue
            for x in range(len(new_reached_goals)):
                if new_reached_goals[x] and not self._reached_goals[x]:
                    new_g = 0


            new_pushed_disks = self._pushed_disks.copy()
            new_pushed_disks.append(disk_being_pushed)
            #use A* where g = full path length
            return PQState(self._map,new_vehicle_pose,self._vehicle_pose,new_disk_positions,self._vehicle_path,new_disk_paths,new_reached_goals,disk_being_pushed,new_pushed_disks,self._RRT,self._g+gValue)
           
        return False


    def getContinuousAnglePushState(self,curr_disk_pos,closest_goal,disk_being_pushed):
        v = BasicGeometry.vec_from_points(closest_goal,curr_disk_pos)
        phi = BasicGeometry.vector_angle(v)
        push_point = Vehicle(curr_disk_pos[0]+2*self._map.disk_radius*math.cos(phi),curr_disk_pos[1]+2*self._map.disk_radius*math.sin(phi),(math.degrees(phi)-180)%360)
        if self._RRT.canConnectPushPoint(push_point,curr_disk_pos):
            (new_disk_pos,new_vehicle_pose) = Pushing.continuousPushDistance(push_point,curr_disk_pos,BasicGeometry.vec_mag(v),self._map)
            if not (curr_disk_pos[0] == new_disk_pos[0] and curr_disk_pos[1] == new_disk_pos[1]):
                gValue = BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y))
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
                new_g = self._g + gValue
                for x in range(len(new_reached_goals)):
                    if new_reached_goals[x] and not self._reached_goals[x]:
                        new_g = 0

                new_pushed_disks = self._pushed_disks.copy()
                new_pushed_disks.append(disk_being_pushed)
                #use A* where g = full path length
                return PQState(self._map,new_vehicle_pose,self._vehicle_pose,new_disk_positions,self._vehicle_path,new_disk_paths,new_reached_goals,disk_being_pushed,new_pushed_disks,self._RRT,new_g)
           
        return False

    def getResultingStates(self,axis):
        resultingStates = []
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            if not self.diskInGoal(curr_disk_pos):
                (closest_goal,found) = self.getClosestGoalToPushLine(curr_disk_pos)
                if found and BasicGeometry.ptDist(closest_goal,curr_disk_pos) < 2.5*self._map.disk_radius:
                    #attempt continuous angle push
                    continousPushState = self.getContinuousAnglePushState(curr_disk_pos,closest_goal,self._disk_being_pushed)
                    if continousPushState != False:
                        resultingStates.append(continousPushState)
                        print("Added continuous angle push state")
                push_point = self._vehicle_pose
                pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._disk_being_pushed,0)
                if pushedState != False:
                    resultingStates.append(pushedState)
                    print("Added pushing state from current pose")
           
                # next consider navigating to a different push point on the current disk
                new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius,self._vehicle_pose.theta)
                for push_point in new_push_points:
                    if self._RRT.canConnectPushPoint(push_point,curr_disk_pos):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._disk_being_pushed,BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y)))
                        if pushedState != False:
                            resultingStates.append(pushedState)
                            print("Added pushing state from new push point on same disk")
                       
                    
             
        # finally consider navigating to the push points of all other disks
        for i in range(len(self._disk_positions)):
            if i == self._disk_being_pushed:
                continue
            curr_disk_pos = self._disk_positions[i]
            if not self.diskInGoal(curr_disk_pos):
                (closest_goal,found) = self.getClosestGoalToPushLine(curr_disk_pos)
                if found and BasicGeometry.ptDist(closest_goal,curr_disk_pos) < 2.5*self._map.disk_radius:
                    #attempt continuous angle push
                    continousPushState = self.getContinuousAnglePushState(curr_disk_pos,closest_goal,i)
                    if continousPushState != False:
                        resultingStates.append(continousPushState)
                        print("Added continuous angle push state new disk")
                new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius)
                for push_point in new_push_points:
                    if self._RRT.canConnectPushPoint(push_point,curr_disk_pos):
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
        #add all final disk positions to their path
        for z in range(len(self._disk_positions)):
            self._disk_paths[z].append(self._disk_positions[z])


        disk_pos_indices = [0]*len(self._disk_paths)
        for i in range(len(self._vehicle_path)):
            #get and plot vehicle position
            curr_path = self._vehicle_path[i]
            for j in range(len(curr_path)-1):
                curr_pose = curr_path[j]
                next_pose = curr_path[j+1]
                if j == len(curr_path)-2 and i<len(self._vehicle_path):
                    #push the disk
                    disk_being_pushed = self._pushed_disks[i]
                    temp = disk_pos_indices[disk_being_pushed]
                    disk_pos_indices[disk_being_pushed] = 0
                    leftList = self._disk_paths[:disk_being_pushed]
                    rightList = self._disk_paths[disk_being_pushed+1:]
                    edge_path = self.generatePosesAlongEdge(curr_pose,next_pose)
                    # push along a continuous path
                    # deduce where the disk is from the vehicle pose
                    for k in range(len(edge_path)):
                        exact_pose = edge_path[k]
                        disk_exact_point = [[[exact_pose.x+2*self._map.disk_radius*math.cos(math.radians(exact_pose.theta)),exact_pose.y+2*self._map.disk_radius*math.sin(math.radians(exact_pose.theta))]]]
                        fig, ax = plt.subplots(1,1)
                   
                        ax = self._map.displayMap(ax,edge_path[k],leftList+disk_exact_point+rightList,disk_pos_indices)
                        fig.canvas.draw()       # draw the canvas, cache the renderer
                        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                        tp = fig.canvas.get_width_height()[::-1]
                        newtp = (tp[0]*2,tp[1]*2)
                        image  = image.reshape(newtp + (3,))
                        if i == len(self._vehicle_path)-1 and k == len(edge_path) - 1:
                            for t in range(175):
                                solution_images.append(image)
                        else:
                            solution_images.append(image)
                        plt.close(fig)

                    disk_pos_indices[disk_being_pushed] = temp + 1
               
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

        
        plt.close("all")
        return solution_images