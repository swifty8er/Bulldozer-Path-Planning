import numpy as np
import cv2
import math

from Map import Map

class Maps:
    #Creates a collection of Map classes based on the Microban Level Text File

    @staticmethod
    def __turnLeft(ori):
            left_ori = np.copy(ori)
            if (left_ori[0] != 0):
                left_ori[1] = left_ori[0]
                left_ori[0] = 0
            else:
                left_ori[0] = -left_ori[1]
                left_ori[1] = 0
            return left_ori

    @staticmethod
    def __turnRight(ori):
        right_ori = np.copy(ori)
        if (right_ori[0] != 0):
            right_ori[1] = -right_ori[0]
            right_ori[0] = 0
        else:
            right_ori[0] = right_ori[1]
            right_ori[1] = 0
        return right_ori

    @staticmethod
    def __ifIndexIsValid(index, max_i, max_j):
        if ((index[0] < 0) or (index[1] < 0) or (index[0] >= max_i) or (index[1] >= max_j)):
            valid = False
        else:
            valid = True
        return valid

    @staticmethod
    def __isIndexInPath(path, index):
        in_path = False
        i = 0
        while ((i < len(path)) and (in_path == False)):
            if ((path[i][0] == index[0]) and (path[i][1] == index[1])):
                in_path = True
            i += 1
        return in_path


    def __init__(self):
        raw_mb = open("C:/Users/cBrak/Documents/UNSW2020/thesis/Bulldozer-Path-Planning/Python/Bulldozer Path Planning/Bulldozer Path Planning/new_maps.txt", "r")
        self._test_maps = []
        if raw_mb.readable():
            self._test_maps = self.LoadElliottTestMaps(raw_mb)
        else:
            print("File not readable")
        raw_mb.close()


    def LoadLevels(self,raw_mb):
        curr_max_str_len = 0
        num_str_in_level = 0
        curr_level_rows = []
        Levels = []
        for raw_mb_line in raw_mb.readlines():
            if "Level" not in raw_mb_line and raw_mb_line != "" and raw_mb_line != " " and raw_mb_line != "\n":
                #find the biggest string in the current level
                if curr_max_str_len < len(raw_mb_line)-1:
                    curr_max_str_len = len(raw_mb_line)-1
                #collect all the strings for this level
                curr_level_rows.append(raw_mb_line.replace("\n", ""))
                num_str_in_level += 1
            elif "Level" in raw_mb_line and curr_max_str_len != 0:
                #Found all strings for a level put in 2d array
                Level = [None]*num_str_in_level
                for index in range(num_str_in_level):
                    tmp_list = list(curr_level_rows[index])
                    #add wall string characters to outside space
                    i = 0
                    while i < len(tmp_list) and tmp_list[i] != "#":
                        tmp_list[i] = "#"
                        i += 1
                    Level[index] = tmp_list
                    #Add wall string character to the end of the list to create rect 2d array
                    if len(curr_level_rows[index]) < curr_max_str_len:
                        for j in range(0,(curr_max_str_len-len(curr_level_rows[index]))):
                            Level[index].append("#")
                Levels.append(Level)
                curr_max_str_len = 0
                num_str_in_level = 0
                curr_level_rows.clear()
        if curr_max_str_len != 0:
            #Found all strings for a level put in 2d array
            Level = [None]*num_str_in_level
            for index in range(num_str_in_level):
                tmp_list = list(curr_level_rows[index])
                #add wall string characters to outside space
                i = 0
                while i < len(tmp_list) and tmp_list[i] != "#":
                    tmp_list[i] = "#"
                    i += 1
                Level[index] = tmp_list
                #Add empty string character to the end of the list to create rect 2d array
                if len(curr_level_rows[index]) < curr_max_str_len:
                    for j in range(0,(curr_max_str_len-len(curr_level_rows[index]))):
                        Level[index].append("#")
            Levels.append(Level)
        return Levels

    def GetStartIJForLevel(self,curr_level):
        start_j = 0
        found = False
        while (start_j < len(curr_level[0])) and not found:
            start_i = 0
            while (start_i < len(curr_level)) and not found:
                if (curr_level[start_i][start_j] != '#'):
                    found = True
                    start_i -= 1
                    start_j -= 1
                start_i += 1
            start_j += 1
        return (start_i,start_j)

    def GetOutlineForLevel(self,curr_level,start_i,start_j,max_i,max_j):
        #Trace the boundary
        first_node = (start_i, start_j)
        #find first curr node after first step
        curr_node = np.array([start_i, start_j])
        first = True
        curr_ori = np.array([0,1])
        curr_outline = [np.array([0,max_i-1-start_i]), np.array([1,max_i-1-start_i])]
        k = 1
        #Do a wall hug algorithm to find the outline, only search NESW in that order
        while ((first == True) or ((first_node[0] != curr_node[0]) or (first_node[1] != curr_node[1]))):
            #find which adjacent nodes are valid
            left_ori = self.__turnLeft(curr_ori)
            curr_left = curr_node + left_ori
            curr_forward = curr_node + curr_ori
            #if there is a wall on the left and the check index is valid
            if ((self.__ifIndexIsValid(curr_left, max_i, max_j)) and (curr_level[curr_left[0]][curr_left[1]] == '#')):
                #if there is no wall in front and check index is valid
                if ((self.__ifIndexIsValid(curr_forward, max_i, max_j)) and (curr_level[curr_forward[0]][curr_forward[1]] != '#')):
                    #move forward
                    curr_node = curr_forward
                    first = False
                    #convert to cartesian cooridinate direction
                    xy_direction = np.array([0,0])
                    xy_direction[0] = curr_ori[1]
                    xy_direction[1] =  -curr_ori[0]

                    curr_outline.append(curr_outline[k] + xy_direction)
                    k += 1
                else:
                    #turn right on the spot
                    curr_ori = self.__turnRight(curr_ori)
                    #convert to cartesian cooridinate direction
                    xy_direction = np.array([0,0])
                    xy_direction[0] = curr_ori[1]
                    xy_direction[1] =  -curr_ori[0]


                    curr_outline.append(curr_outline[k] + xy_direction)
                    k += 1
            else:
                #turn left
                curr_node = curr_left
                curr_ori = left_ori
                first = False
                #convert to cartesian cooridinate direction
                xy_direction = np.array([0,0])
                xy_direction[0] = curr_ori[1]
                xy_direction[1] =  -curr_ori[0]

                curr_outline[k] = curr_outline[k-1] + xy_direction
        return curr_outline

    def BinariseImage(self,curr_level,max_i,max_j):
        #Convert GridMap into Binary Image
        BI = np.zeros((max_i,max_j))
        for p in range(max_i):
            for q in range(max_j):
                if curr_level[p][q] == "#":
                    BI[p][q] = 1
        return BI

    def GetObstaclesList(self,max_i,max_j,nb_components,output):
        obs_list = []
        if nb_components > 2:
            for r in range(2,nb_components):
                curr_obs = []
                for p in range(max_i):
                    for q in range(max_j):
                        if output[p][q] == r:
                            curr_obs.append([p,q])
                obs_list.append(curr_obs)
        return obs_list

    def GetObstacles(self,curr_level,mcurr_outline,obs_list,min_x,max_y,max_i,max_j):
        obstacles = []
        #setting up Outlines to fill later
        #Now find the outlines of the obstalces
        #Find the starting point for the boundary


        for p in range(len(obs_list)):
            if (len(obs_list[p]) == 1):
                #obstacle is just a box
                curr_obs_outline = [np.array([obs_list[p][0][1]-min_x-1,max_y+1-obs_list[p][0][0]])]
                curr_obs_outline.append(curr_obs_outline[0] + [1,0])
                curr_obs_outline.append(curr_obs_outline[1] + [0,-1])
                curr_obs_outline.append(curr_obs_outline[2] + [-1,0])
                curr_obs_outline.append(curr_obs_outline[3] + [0,1])
            else:
                #Trace the boundary of the obstacles
                curr_obs_outline = [np.array([obs_list[p][0][1]-min_x-1,max_y+1-obs_list[p][0][0]])]
                first_node = np.array([obs_list[p][0][0], obs_list[p][0][1]])
                #find first curr node after first step
                curr_node = first_node.copy()
                first = True
                visited = [first_node.copy()]
                curr_ori = np.array([-1,0])
                num_obs_blocks = len(obs_list[p])
                k = 0
                #Do a wall hug algorithm to find the obstacle outline, only search NESW in that order
                while ((num_obs_blocks != len(visited)) or ((first_node[0] != curr_node[0]) or (first_node[1] != curr_node[1]))):
                    #find which adjacent nodes are valid
                    left_ori = self.__turnLeft(curr_ori)
                    curr_left = curr_node + left_ori
                    curr_forward = curr_node + curr_ori
                    #if there is no wall on the left and check index is valid
                    if ((self.__ifIndexIsValid(curr_left, max_i, max_j)) and (curr_level[curr_left[0]][curr_left[1]] != '#')):
                        #if there is a wall in front and check index is valid
                        if ((self.__ifIndexIsValid(curr_forward, max_i, max_j)) and (curr_level[curr_forward[0]][curr_forward[1]] == '#')):
                            #move forward
                            curr_node = curr_forward
                            first = False
                            if (not self.__isIndexInPath(visited, curr_node)):
                                visited.append(curr_node.copy())

                            #convert to cartesian cooridinate direction
                            xy_direction = np.array([0,0])
                            xy_direction[0] = curr_ori[1]
                            xy_direction[1] =  -curr_ori[0]

                            curr_obs_outline.append(curr_obs_outline[k] + xy_direction)
                            k += 1
                        else:
                            #turn right on the spot
                            curr_ori = self.__turnRight(curr_ori)
                            #convert to cartesian cooridinate direction
                            xy_direction = np.array([0,0])
                            xy_direction[0] = curr_ori[1]
                            xy_direction[1] =  -curr_ori[0]

                            curr_obs_outline.append(curr_obs_outline[k] + xy_direction)
                            k += 1

                    else:
                        #turn left
                        curr_node = curr_left
                        curr_ori = left_ori
                        first = False
                        if (not self.__isIndexInPath(visited, curr_node)):
                            visited.append(curr_node.copy())

                        #convert to cartesian cooridinate direction
                        xy_direction = np.array([0,0])
                        xy_direction[0] = curr_ori[1]
                        xy_direction[1] =  -curr_ori[0]

                        curr_obs_outline[k] = curr_obs_outline[k-1] + xy_direction
                        #curr_obs_outline(k,:) = [];
                        #k = k - 1;
                        #curr_obs_outline(k+1,:) = curr_obs_outline(k,:) + xy_direction;
                        #k = k + 1;
            if ((curr_obs_outline[0][0] !=  curr_obs_outline[-1][0]) or (curr_obs_outline[0][1] !=  curr_obs_outline[-1][1])):
                curr_obs_outline.append(curr_obs_outline[0])

            obstacles.append(curr_obs_outline)
        return obstacles

    def GetMinyMaxx(self,curr_outline):
        min_y = math.inf
        max_x = 0
        for coord in curr_outline:
            if coord[1] < min_y:
                min_y = coord[1]
            if coord[0] > max_x:
                max_x = coord[0]
        return (min_y,max_x)


    def FindVehicleDiskAndGoalPos(self,curr_level,max_i,max_j,min_x,max_x,min_y,max_y):
        #find vehicle, disk and goal positions
        goal_pos_xy = []
        initial_vehicle_pos_xy = []
        initial_disk_pos_xy = []
        for j in range (max_i):
            for k in range(max_j):
                if (curr_level[j][k] == '.'):
                    #goal
                    goal_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
                elif (curr_level[j][k] == '@'):
                    #vehicle
                    initial_vehicle_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
                elif (curr_level[j][k] == '$'):
                    #disk
                    initial_disk_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
                elif (curr_level[j][k] == '*'):
                    #disk on top of goal
                    initial_disk_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
                    goal_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
                elif (curr_level[j][k] == '+'):
                    #vehicle on top of goal
                    initial_vehicle_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
                    goal_pos_xy.append([k-min_x-0.5,max_y+0.5-j]) #converted
        return (initial_vehicle_pos_xy,initial_disk_pos_xy,goal_pos_xy)

    def GetMinxMaxy(self,curr_outline):
        min_x = math.inf
        max_y = 0
        for coord in curr_outline:
            if coord[0] < min_x:
                min_x = coord[0]
            if coord[1] > max_y:
                max_y = coord[1]
        return (min_x,max_y)

    def LoadElliottTestMaps(self,raw_mb):
        lines = []
        goals = []
        disks = []
        obstacles = []
        testMaps = []
        mapNum = 0
        numLines = -1000
        for line in raw_mb.readlines():
            if len(line.split()) > 1 and line.split()[0] == "Level":
                i = 0
                mapNum = int(line.split()[1])
            if i == 1:
                numLines = int(line.strip())
            if i == 2:
                lineSegments = line.split()
                j = 0
                for parts in lineSegments:
                    j = 0
                    lineList = []
                    l = []
                    for part in parts.split(','):
                        if j==0:
                            l.append(float(part.strip().strip('[')))
                        elif j== 1:
                            l.append(float(part.strip().strip(']')))
                            lineList.append(l)
                        elif j==2:
                            l = []
                            l.append(float(part.strip().strip('[')))
                        elif j==3:
                            l.append(float(part.strip().strip(']')))
                            lineList.append(l)
                        j+=1
                    lines.append(lineList)
            elif i==3:
                goalPos = line.split()
                for goal in goalPos:
                    x = float(goal.split(',')[0].strip().strip('['))
                    y = float(goal.split(',')[1].strip().strip(']'))
                    newGoal = [x,y]
                    goals.append(newGoal)
            elif i==4:
                diskPos = line.split()
                for disk in diskPos:
                    x = float(disk.split(',')[0].strip().strip('['))
                    y = float(disk.split(',')[1].strip().strip(']'))
                    newDisk = [x,y]
                    disks.append(newDisk)
            elif i==5:
                vehiclePos = line
                x = float(vehiclePos.split(',')[0].strip().strip('['))
                y = float(vehiclePos.split(',')[1].strip().strip(']'))
                v = [x,y]
            elif i>0 and  i <= numLines + 1:
                obstacleSegments = line.split()
                j = 0
                for parts in obstacleSegments:
                    j = 0
                    obstacleList = []
                    obs = []
                    for part in parts.split(','):
                        if j==0:
                            obs.append(float(part.strip().strip('[')))
                        elif j== 1:
                            obs.append(float(part.strip().strip(']')))
                            obstacleList.append(obs)
                        elif j==2:
                            obs = []
                            obs.append(float(part.strip().strip('[')))
                        elif j==3:
                            obs.append(float(part.strip().strip(']')))
                            obstacleList.append(obs)
                        j+=1
                    if len(obstacleList) > 0:
                        obstacles.append(obstacleList)
            if i == numLines + 1:
                (minx,maxx,miny,maxy) = self.getMinMax(lines)
                newMap = Map(mapNum,minx,miny,maxx,maxy,1,lines,obstacles,0.45,0.45,goals,v,disks)
                testMaps.append(newMap)
                lines = []
                goals = []
                disks = []
                obstacles = []

            i+=1

        return testMaps


    def getMinMax(self,lines):
        min_x = math.inf
        min_y = math.inf
        max_x = 0
        max_y = 0
        for line in lines:
            (x1,y1) = line[0]
            (x2,y2) = line[1]
            if x2>x1:
                if x1<min_x:
                    min_x = x1
                if x2>max_x:
                    max_x = x2
            elif x2<x1:
                if x2<min_x:
                    min_x = x2
                if x1>max_x:
                    max_x = x1
            if y2>y1:
                if y1<min_y:
                    min_y = y1
                if y2>max_y:
                    max_y = y2
            elif y2<y1:
                if y2<min_y:
                    min_y = y2
                if y1>max_y:
                    max_y = y1

        return (min_x,max_x,min_y,max_y)

    def LoadTestMaps(self,raw_mb):
        testMaps = []
        Levels = self.LoadLevels(raw_mb)
        #now we have a list of 2d arrays with all the characters for each level
        i = 1
        for curr_level in Levels:
            start_i, start_j = self.GetStartIJForLevel(curr_level)
            
            
            max_i = len(curr_level)
            max_j = len(curr_level[0])
            curr_outline = self.GetOutlineForLevel(curr_level,start_i,start_j,max_i,max_j)
            BI  = self.BinariseImage(curr_level,max_i,max_j)
            #Find any obstacles inside the boundary
            BI = np.uint8(BI)
            # Perform the operation
            nb_components, output, _, _ = cv2.connectedComponentsWithStats(BI,connectivity=4)
            obs_list = self.GetObstaclesList(max_i,max_j,nb_components,output)
            min_x,max_y = self.GetMinxMaxy(curr_outline)
            obstacles = self.GetObstacles(curr_level,curr_outline,obs_list,min_x,max_y,max_i,max_j)
            #print("Level", i + 1, "Complete")
            min_y,max_x = self.GetMinyMaxx(curr_outline)
            
            initial_vehicle_pos_xy,initial_disk_pos_xy,goal_pos_xy = self.FindVehicleDiskAndGoalPos(curr_level,max_i,max_j,min_x,max_x,min_y,max_y)
            curr_map = Map(i, min_x, min_y, max_x, max_y, 1, curr_outline, obstacles, 0.45, 0.45, goal_pos_xy, initial_vehicle_pos_xy, initial_disk_pos_xy)
            testMaps.append(curr_map)
            i+=1

        return testMaps

    @property
    def test_maps(self):
        return self._test_maps