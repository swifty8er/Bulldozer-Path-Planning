class PushState:
    def __init__(self,f,disk_pos,statuses,g):
        self._f = f
        self._disk_pos = disk_pos
        self._statuses = statuses
        self._g = g

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
        for i in range(num_steps):
            r = random.uniform(0.0,max_push_distance)
            new_disk_pos = (self._disk_pos[0]+r*math.cos(math.radians(push_point.theta)),self._disk_pos[1]+r*math.sin(math.radians(push_point.theta)))
            dist = self.manhattanDistance(new_disk_pos,closestGoal)
            if dist < min_dist:
                min_dist = dist
                bestPush = new_disk_pos

        if bestPush == None:
            raise Exception("Failed to push disk towards closest goal")
        return bestPush

    def getClosestGoal(self,goalPositions):
        min_dist = math.inf
        closestGoal = None
        for pos in goalPositions:
            dist = self.manhattanDistance(self._disk_pos,pos)
            if dist < min_dist:
                closestGoal = pos
                min_dist = dist

        if closestGoal == None:
            raise Exception("Could not find closest goal to push to")
        return closestGoal

    def manhattanDistance(self,pos1,pos2):
        (x1,y1) = pos1
        (x2,y2) = pos2
        return abs(x1-x2) + abs(y1-y2)


