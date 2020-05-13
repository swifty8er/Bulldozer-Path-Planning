import math
import random
from BasicGeometry import BasicGeometry
from Vehicle import Vehicle

class Pushing:
    @staticmethod
    def pushDisk(push_point,curr_disk_pos,closestGoal,map,max_distance=(0.4*math.pi)/4.0,num_steps=50):
        bestPush = None
        min_dist = math.inf
        found = False
        if BasicGeometry.ptDist(closestGoal,curr_disk_pos) < max_distance:
            upperBound = BasicGeometry.ptDist(closestGoal,curr_disk_pos)
            lowerBound = 0.0
            r = random.uniform(lowerBound,upperBound)
        else:
            r = max_distance
        for i in range(num_steps):
            new_disk_pos = (curr_disk_pos[0]+r*math.cos(math.radians(push_point.theta)),curr_disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
            dist = BasicGeometry.manhattanDistance(new_disk_pos,closestGoal)
            if dist < min_dist and not Pushing.pushingCollision(push_point,new_disk_pos,map):
                min_dist = dist
                new_vehicle_pose = Vehicle(push_point.x+r*math.cos(math.radians(push_point.theta)),push_point.y+r*math.sin(math.radians(push_point.theta)),push_point.theta)
                bestPush = (new_disk_pos,new_vehicle_pose)
                found = True

        if not found:
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
        return False


    # Find the push points for the vehicle to push the given disk with its boundary
    @staticmethod
    def getPushPoints(disk_pos,disk_radius,curr_heading=-1):
        push_points = []
        angle = 0
        while (angle<=2*math.pi):
            push_point = (disk_pos[0]+2*disk_radius*math.cos(angle),disk_pos[1]+2*disk_radius*math.sin(angle))
            heading = math.degrees(BasicGeometry.vector_angle(BasicGeometry.vec_from_points(push_point,disk_pos)))
            if round(heading,0) == curr_heading:
                angle+= (math.pi/6.0)
                continue

            push_pose = Vehicle(push_point[0],push_point[1],heading%360)
            push_points.append(push_pose)
            angle += (math.pi/6.0)
        return push_points

