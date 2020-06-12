import math
import queue
import bezier
import time
import copy
import numpy as np
from BasicGeometry import BasicGeometry
from BezierLib import BezierLib
from Pushing import Pushing
from Vehicle import Vehicle
from TranspositionTable import TranspositionTable
from matplotlib import pyplot as plt


class PQState:
    def __init__(self,map,vehicle_pose,previous_pose,curr_disk_positions,vehicle_path,past_disk_positions,reached_goals,disk_being_pushed,pushed_disks,rrt,g):
        self._map = map
        self._vehicle_pose = vehicle_pose
        self._previous_pose = previous_pose
        self._curr_disk_positions = curr_disk_positions
        self._vehicle_path = vehicle_path
        self._past_disk_positions = past_disk_positions
        self._reached_goals = reached_goals
        self._pushed_disks = pushed_disks
        self._RRT = rrt
        self._disk_being_pushed = disk_being_pushed #index of disk being pushed -1 = no disk
        self._g = g
        self._f = self.calculateHeuristicValue() #greedy search for now

    
    @property
    def f(self):
        return self._f

    @property
    def vehicle_pose(self):
        return self._vehicle_pose

    @property
    def curr_disk_positions(self):
        return self._curr_disk_positions

    def __lt__(self,other):
        return self.f < other.f


    def plotState(self,ax,line_width=2):
        self._map.plotMapBoundaryObstacles(ax)
        for disk_pos in self._curr_disk_positions:
            disk_circle = BasicGeometry.circlePoints(disk_pos, self._map.disk_radius, 25)
            ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
        self.drawVehiclePose(ax)


    # a hacky zobrist hash - does not use random numbers
    def __hash__(self):
        h = 0
        for pos in self._curr_disk_positions:
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
        for disk in self._curr_disk_positions:
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
        return self._past_disk_positions[-1]


    def connectBezierCurveBetweenTwoPoses(self,pose1,pose2,curr_disk_positions,starting_num_iterations=500,max_degree=5):
        degree = 3
        iterations = starting_num_iterations
        while degree <= max_degree:
            bezier_curve = BezierLib.getBestBezierCurveConnectionBetweenTwoPoses(pose1,pose2,self._map,curr_disk_positions,degree,iterations,25)
            if bezier_curve != False:
                return (True,bezier_curve,"F")
            rev_p1 = Vehicle(pose1.x,pose1.y,(pose1.theta+180)%360)
            rev_p2 = Vehicle(pose2.x,pose2.y,(pose2.theta+180)%360)
            inv_bezier = BezierLib.getBestBezierCurveConnectionBetweenTwoPoses(rev_p1,rev_p2,self._map,curr_disk_positions,degree,iterations,25)
            if inv_bezier != False:
                return (True,inv_bezier,"R")
            degree += 1
            iterations *= 1.5
        return (False,None,None)

    def bezierSmoothSolutionPath(self,ax=False):
        i = 0
        final_path = []
        for path in self._vehicle_path:
            if len(path) > 2:
                curr_disk_pos = self._past_disk_positions[i]
                finalPose = [path[-1]]
                new_path = self.bezierSmoothPath(path[:-1],curr_disk_pos,ax) + finalPose
            else:
                new_path = path
            final_path.append(new_path)
            i+=1

        self._vehicle_path = final_path


    def bezierSmoothPath(self,path,curr_disk_pos,ax1=False):
        startIndex = 0
        endIndex = len(path) - 1
        while endIndex > startIndex:
            pose1 = path[startIndex]
            pose2 = path[endIndex]
            if pose2 in self._RRT.tree[pose1]:
                startIndex = endIndex
                endIndex = len(path)-1
            else:
                (successful,bezier_curve,direction) = self.connectBezierCurveBetweenTwoPoses(pose1,pose2,curr_disk_pos)
                if successful:
                    if direction == "F":
                        opp_direction = "R"
                    else:
                        opp_direction = "F"
                    self._RRT.addEdge(pose2,pose1,(bezier_curve,direction),(BezierLib.getInverseCurve(bezier_curve),opp_direction))
                    path = path[:startIndex+1] + path[endIndex:]
                    
                    startIndex = endIndex
                    endIndex = len(path)-1
                else:
                    endIndex -= 1
        return path


    def growRRTAndConnectToPushPoint(self,axis=False):
        push_point = list(self._RRT.tree[self._vehicle_pose].keys())[0]
        return self._RRT.dynamicallyGrowSubRRTAndConnectToPushPoint(self._previous_pose,push_point,self._curr_disk_positions,axis)

    def connectToPreviousPose(self,axis=False):
        rewiring_candidates = []
        if self._previous_pose == None:
            return (True,[])
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
                self._vehicle_path = copy.deepcopy(self._vehicle_path)
                self._vehicle_path.append(path)
                return (True,[])
            if len(path)>0:
                curr_disk_positions = self.rollBackDiskPush()
            else:
                curr_disk_positions = self._curr_disk_positions
            if pose not in visitedNodes:
                visitedNodes[pose] = True
                new_path = path.copy()
                new_path.append(pose)
                for next_pose in self._RRT.tree[pose]:
                    if self._RRT.edgeCollidesWithDirtPile(pose,next_pose,self._RRT.tree[pose][next_pose],curr_disk_positions):
                        rewiring_candidates.append(pose)
                    elif not next_pose in visitedNodes:
                        new_g = self.getEdgeLength(pose,next_pose)
                        new_state = (g+new_g+next_pose.EuclideanDistance(previousPose),next_pose,new_path,g+new_g) #change this to use the arc path length
                        pq.put(new_state)
                          

        return (False,rewiring_candidates)


    def rewireRRT(self,candidates,ax=False):
        for candidate in candidates:
            self._RRT.rewireNode(candidate,self._curr_disk_positions,ax)

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
        plt.cla()
        self.plotState(ax)
        plt.draw()
        plt.pause(0.1)
        plt.show(block=False)
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
                if (BasicGeometry.ptDist(disk_pos,goal_pos)) < 0.075:
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
            new_disk_positions = copy.deepcopy(self._curr_disk_positions)
            new_disk_positions[disk_being_pushed] = new_disk_pos
            #vehicle_path.append(push_point)
            #vehicle_path.append(new_vehicle_pose)
            #new_vehicle_paths = copy.deepcopy(self._vehicle_path)
            #new_vehicle_paths.append(vehicle_path)
            new_past_disk_positions = copy.deepcopy(self._past_disk_positions)
            new_past_disk_positions.append(self._curr_disk_positions)
            new_reached_goals = self.determineGoalsReached(new_disk_positions)
            new_g = self._g + gValue
            new_pushed_disks = self._pushed_disks.copy()
            new_pushed_disks.append(disk_being_pushed)
            #use A* where g = full path length
            return PQState(self._map,new_vehicle_pose,self._vehicle_pose,new_disk_positions,self._vehicle_path,new_past_disk_positions,new_reached_goals,disk_being_pushed,new_pushed_disks,self._RRT,self._g+gValue)
           
        return False


    def getContinuousAnglePushState(self,curr_disk_pos,closest_goal,disk_being_pushed,ax=False):
        v = BasicGeometry.vec_from_points(closest_goal,curr_disk_pos)
        phi = BasicGeometry.vector_angle(v)
        push_point = Vehicle(curr_disk_pos[0]+2*self._map.disk_radius*math.cos(phi),curr_disk_pos[1]+2*self._map.disk_radius*math.sin(phi),(math.degrees(phi)-180)%360)
        if self._RRT.connectPushPoint(push_point,curr_disk_pos,self._curr_disk_positions):
            (new_disk_pos,new_vehicle_pose) = Pushing.continuousPushDistance(push_point,curr_disk_pos,BasicGeometry.vec_mag(v),self._map)
            if not (curr_disk_pos[0] == new_disk_pos[0] and curr_disk_pos[1] == new_disk_pos[1]):
                gValue = BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y))
                distance = push_point.EuclideanDistance(new_vehicle_pose)
                new_edge = (distance,0,"F")
                inv_edge = (distance,0,"R")
                self._RRT.addEdge(new_vehicle_pose,push_point,new_edge,inv_edge)
                new_disk_positions = copy.deepcopy(self._curr_disk_positions)
                new_disk_positions[disk_being_pushed] = new_disk_pos
                #vehicle_path.append(push_point)
                #vehicle_path.append(new_vehicle_pose)
                #new_vehicle_paths = copy.deepcopy(self._vehicle_path)
                #new_vehicle_paths.append(vehicle_path)
                new_past_disk_positions = copy.deepcopy(self._past_disk_positions)
                new_past_disk_positions.append(self._curr_disk_positions)
                new_reached_goals = self.determineGoalsReached(new_disk_positions)
                new_g = self._g + gValue
                new_pushed_disks = self._pushed_disks.copy()
                new_pushed_disks.append(disk_being_pushed)
                #use A* where g = full path length
                return PQState(self._map,new_vehicle_pose,self._vehicle_pose,new_disk_positions,self._vehicle_path,new_past_disk_positions,new_reached_goals,disk_being_pushed,new_pushed_disks,self._RRT,new_g)
           
        return False

    def getResultingStates(self,axis):
        resultingStates = []
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._curr_disk_positions[self._disk_being_pushed]
            if not self.diskInGoal(curr_disk_pos):
                (closest_goal,found) = self.getClosestGoalToPushLine(curr_disk_pos)
                if found and BasicGeometry.ptDist(closest_goal,curr_disk_pos) < 2.5*self._map.disk_radius:
                    #attempt continuous angle push
                    continousPushState = self.getContinuousAnglePushState(curr_disk_pos,closest_goal,self._disk_being_pushed,axis)
                    if continousPushState != False:
                        resultingStates.append(continousPushState)
                push_point = self._vehicle_pose
                pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._disk_being_pushed,0)
                if pushedState != False:
                    resultingStates.append(pushedState)
           
                # next consider navigating to a different push point on the current disk
                new_push_points = Pushing.getPushPointsFromHeatmap(curr_disk_pos,closest_goal,self._map,self._vehicle_pose.theta)
                for push_point in new_push_points:
                    if self._RRT.connectPushPoint(push_point,curr_disk_pos,self._curr_disk_positions):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._disk_being_pushed,BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y)))
                        if pushedState != False:
                            resultingStates.append(pushedState)
                       
                    
             
        # finally consider navigating to the push points of all other disks
        for i in range(len(self._curr_disk_positions)):
            if i == self._disk_being_pushed:
                continue
            curr_disk_pos = self._curr_disk_positions[i]
            if not self.diskInGoal(curr_disk_pos):
                (closest_goal,found) = self.getClosestGoalToPushLine(curr_disk_pos)
                if found and BasicGeometry.ptDist(closest_goal,curr_disk_pos) < 2.5*self._map.disk_radius:
                    #attempt continuous angle push
                    continousPushState = self.getContinuousAnglePushState(curr_disk_pos,closest_goal,i,axis)
                    if continousPushState != False:
                        resultingStates.append(continousPushState)
                new_push_points = Pushing.getPushPointsFromHeatmap(curr_disk_pos,closest_goal,self._map,self._vehicle_pose.theta)
                for push_point in new_push_points:
                    if self._RRT.connectPushPoint(push_point,curr_disk_pos,self._curr_disk_positions):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,i,BasicGeometry.manhattanDistance((self._vehicle_pose.x,self._vehicle_pose.y),(push_point.x,push_point.y)))
                        if pushedState != False:
                            resultingStates.append(pushedState)
                        
        return resultingStates


    def generatePosesAlongEdge(self,n1,n2,num_steps=12):
        poses = [n1]
        if n1 in self._RRT.tree:
            if n2 in self._RRT.tree[n1]:
                edge = self._RRT.tree[n1][n2]
                if edge != False: #this check should no longer be necessary
                    if isinstance(edge[0],bezier.curve.Curve):
                        (bezierEdge,direction) = edge
                        s = 0.0
                        while s<1.0:
                            point_list = bezierEdge.evaluate(s).tolist()
                            real_point = [point_list[0][0],point_list[1][0]]
                            if s==0:
                                heading = n1.theta
                            else:
                                if direction == "F":
                                    next_list = bezierEdge.evaluate(s+0.04).tolist()
                                else:
                                    next_list = bezierEdge.evaluate(s-0.04).tolist()
                                real_next = [next_list[0][0],next_list[1][0]]
                                heading = math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(real_point,real_next)))
                            
                            newPose = Vehicle(real_point[0],real_point[1],heading)
                            poses.append(newPose)
                            s+=0.04
                        
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

        final_disk_positions = self._past_disk_positions
        final_disk_positions.append(self._curr_disk_positions)
        disk_index = 0
        for i in range(len(self._vehicle_path)):
            #get and plot vehicle position
            curr_path = self._vehicle_path[i]
            curr_disk_positions = final_disk_positions[disk_index]
            for j in range(len(curr_path)-1):
                curr_pose = curr_path[j]
                next_pose = curr_path[j+1]
                if j == len(curr_path)-2 and i<len(self._vehicle_path):
                    #push the disk
                  
                    disk_being_pushed = self._pushed_disks[i] # i = disk_index
                    leftList = curr_disk_positions[:disk_being_pushed]
                    rightList = curr_disk_positions[disk_being_pushed+1:]
                    edge_path = self.generatePosesAlongEdge(curr_pose,next_pose)
                    # push along a continuous path
                    # deduce where the disk is from the vehicle pose
                    for k in range(len(edge_path)):
                        exact_pose = edge_path[k]
                        disk_exact_point = [[exact_pose.x+2*self._map.disk_radius*math.cos(math.radians(exact_pose.theta)),exact_pose.y+2*self._map.disk_radius*math.sin(math.radians(exact_pose.theta))]]
                        fig, ax = plt.subplots(1,1)
                   
                        ax = self._map.displayMap(ax,edge_path[k],leftList+disk_exact_point+rightList)
                        fig.canvas.draw()       # draw the canvas, cache the renderer
                        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                        tp = fig.canvas.get_width_height()[::-1]
                        newtp = (tp[0]*2,tp[1]*2)
                        image  = image.reshape(newtp + (3,))
                        if i == len(self._vehicle_path)-1 and k == len(edge_path) - 1:
                            for t in range(100):
                                solution_images.append(image)
                        else:
                            solution_images.append(image)
                        plt.close(fig)

                    disk_index += 1
               
                else:
                  
                    edge_path = self.generatePosesAlongEdge(curr_pose,next_pose)
                    for k in range(len(edge_path)):

                        fig, ax = plt.subplots(1,1)
                   
                        ax = self._map.displayMap(ax,edge_path[k],curr_disk_positions)
                        fig.canvas.draw()       # draw the canvas, cache the renderer
                        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                        tp = fig.canvas.get_width_height()[::-1]
                        newtp = (tp[0]*2,tp[1]*2)
                        image  = image.reshape(newtp + (3,))
                        solution_images.append(image)
                        plt.close(fig)

        
        plt.close("all")
        return solution_images