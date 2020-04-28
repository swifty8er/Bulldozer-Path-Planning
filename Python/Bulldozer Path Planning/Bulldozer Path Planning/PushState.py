from BasicGeometry import BasicGeometry
import math
import random
class PushState:
    def __init__(self,f,disk_pos,statuses,g):
        self._f = f
        self._disk_pos = disk_pos
        self._statuses = statuses
        self._g = g

    # disk_pos is a point tuple (x,y
    def getDiskPos(self):
        return self._disk_pos

    def getStatuses(self):
        return self._statuses

    def getG(self):
        return self._g

    def diskAtGoal(self,goalPositions):
        for pos in goalPositions:
            if (abs(pos[0] - self._disk_pos[0]) < 0.05) and (abs(pos[1] - self._disk_pos[1]) < 0.05):
                return True
        return False

    def pushDisk(self,push_point,goalPositions,max_push_distance,num_steps):
        closestGoal = self.getClosestGoal(goalPositions)
        bestPush = None
        min_dist = math.inf
        found = False
        for i in range(num_steps):
            r = random.uniform(0.0,max_push_distance)
            new_disk_pos = (self._disk_pos[0]+r*math.cos(math.radians(push_point.theta)),self._disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
            dist = BasicGeometry.manhattanDistance(new_disk_pos,closestGoal)
            if dist < min_dist:
                min_dist = dist
                bestPush = new_disk_pos
                found = True

        if not found:
            raise Exception("Failed to push disk towards closest goal")
        print("Pushed disk from push point (%.2f,%.2f,%.2f) to (%.2f,%.2f)" % (push_point.x,push_point.y,push_point.theta,bestPush[0],bestPush[1]))
        return bestPush

    def getClosestGoal(self,goalPositions):
        min_dist = math.inf
        closestGoal = None
        found = False
        for pos in goalPositions:
            dist = BasicGeometry.manhattanDistance(self._disk_pos,pos)
            if dist < min_dist:
                closestGoal = pos
                min_dist = dist
                found = True
        
        if not found:
            raise Exception("Could not find closest goal")
        return closestGoal
        

   


