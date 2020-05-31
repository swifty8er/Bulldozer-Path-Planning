import math
import random
from BasicGeometry import BasicGeometry
from Vehicle import Vehicle

class Pushing:

    @staticmethod
    def continuousPushDistance(push_point,curr_disk_pos,distance,map):
        new_disk_pos = (curr_disk_pos[0]+distance*math.cos(math.radians(push_point.theta)),curr_disk_pos[1]+distance*math.sin(math.radians(push_point.theta)))
        if not Pushing.pushingCollision(push_point,new_disk_pos,map):
            new_vehicle_pose = Vehicle(push_point.x+distance*math.cos(math.radians(push_point.theta)),push_point.y+distance*math.sin(math.radians(push_point.theta)),push_point.theta)
            return (new_disk_pos,new_vehicle_pose)
        else:
            return (curr_disk_pos,push_point)

    @staticmethod
    def pushDisk(push_point,curr_disk_pos,closestGoal,map,max_distance=(0.4*math.pi)/4.0,num_steps=50):
        bestPush = None
        min_dist = math.inf
        found = False
        if BasicGeometry.ptDist(closestGoal,curr_disk_pos) < max_distance:
            upperBound = BasicGeometry.ptDist(closestGoal,curr_disk_pos)
            lowerBound = 0.0
            r = random.uniform(lowerBound,upperBound)
            for i in range(num_steps):
                new_disk_pos = (curr_disk_pos[0]+r*math.cos(math.radians(push_point.theta)),curr_disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
                dist = BasicGeometry.manhattanDistance(new_disk_pos,closestGoal)
                if dist < min_dist and not Pushing.pushingCollision(push_point,new_disk_pos,map):
                    min_dist = dist
                    new_vehicle_pose = Vehicle(push_point.x+r*math.cos(math.radians(push_point.theta)),push_point.y+r*math.sin(math.radians(push_point.theta)),push_point.theta)
                    bestPush = (new_disk_pos,new_vehicle_pose)
                    found = True
        else:
            r = max_distance
            new_disk_pos = (curr_disk_pos[0]+r*math.cos(math.radians(push_point.theta)),curr_disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
            dist = BasicGeometry.manhattanDistance(new_disk_pos,closestGoal)
            if not Pushing.pushingCollision(push_point,new_disk_pos,map):
                new_vehicle_pose = Vehicle(push_point.x+r*math.cos(math.radians(push_point.theta)),push_point.y+r*math.sin(math.radians(push_point.theta)),push_point.theta)
                bestPush = (new_disk_pos,new_vehicle_pose)
                found = True


        if not found:
            return (curr_disk_pos,push_point)
        # do not push beyond a certain angle
        elif BasicGeometry.ptDist(closestGoal,curr_disk_pos) < 1.5* max_distance and BasicGeometry.vehicleDiskGoalAngle(bestPush[1],bestPush[0],closestGoal) < 140:
            return (curr_disk_pos,push_point)
        else:
            return bestPush


    @staticmethod
    def pushingCollision(push_point,disk_pos,map):
        edges =  map.getMapEdgesAndObstacles()
        newLine = [[push_point.x,push_point.y],[disk_pos[0],disk_pos[1]]]
        for edge in edges:
            if BasicGeometry.doLinesIntersect(newLine,edge):
                return True
            if BasicGeometry.circleArcIntersectsLine(disk_pos,map.disk_radius,edge):
                return True
        return False


    @staticmethod
    def getContinuousPushAngle(curr_disk_pos,closest_goal):
        v = BasicGeometry.vec_from_points(closest_goal,curr_disk_pos)
        phi = BasicGeometry.vector_angle(v)
        return (math.degrees(phi)-180)%360

    # Find the push points for the vehicle to push the given disk with its boundary
    @staticmethod
    def getPushPoints(disk_pos,disk_radius,closest_goal,curr_heading=-1):
        continuousPushAngle = Pushing.getContinuousPushAngle(disk_pos,closest_goal)
        push_points = []
        angle = 0
        while (angle<=2*math.pi):
            push_point = (disk_pos[0]+2*disk_radius*math.cos(angle),disk_pos[1]+2*disk_radius*math.sin(angle))
            heading = (math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(push_point,disk_pos)))%360)
            if round(heading,0) == curr_heading:
                angle+= (math.pi/6.0)
                continue
            elif abs(heading-continuousPushAngle) <=90:
                push_pose = Vehicle(push_point[0],push_point[1],heading%360)
                push_points.append(push_pose)
            angle += (math.pi/6.0)
        return push_points

