import math
import random
from BasicGeometry import BasicGeometry

class Pushing:
    @staticmethod
    def pushDisk(push_point,curr_disk_pos,closestGoal,max_distance=(0.4*math.pi)/4.0,num_steps=50):
        bestPush = None
        min_dist = math.inf
        found = False
        if BasicGeometry.ptDist(closestGoal,curr_disk_pos) < max_distance:
            upperBound = BasicGeometry.ptDist(closestGoal,curr_disk_pos)
            lowerBound = 0.0
        else:
            upperBound = max_distance
            lowerBound = max_distance/2.0
        for i in range(num_steps):
            r = random.uniform(lowerBound,upperBound)
            new_disk_pos = (curr_disk_pos[0]+r*math.cos(math.radians(push_point.theta)),curr_disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
            dist = BasicGeometry.manhattanDistance(new_disk_pos,closestGoal)
            if dist < min_dist and not self.pushingCollision(push_point,new_disk_pos):
                min_dist = dist
                new_vehicle_pose = (push_point.x+r*math.cos(math.radians(push_point.theta)),push_point.y+r*math.sin(math.radians(push_point.theta)))
                bestPush = (new_disk_pos,new_vehicle_pose)
                found = True

        if not found:
            return (curr_disk_pos,push_point)
        else:
            return bestPush


