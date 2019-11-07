import numpy as np
import cv2
import sys

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
        raw_mb = open("C:/Users/User/Documents/Bulldozer Path Planning/Python/Bulldozer Path Planning/Bulldozer Path Planning/Microban Levels.txt", "r")
        self._test_maps = []
        if raw_mb.readable():
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
        #row we have a list of 2d arrays with all the characters for each level
        else:
            print("File not readable")

        raw_mb.close()
        #for i in range(156):
        #    for j in range(len(Levels[i])):
        #        print(Levels[i][j])
        for i in range(len(Levels)):
            #Map out the lines that make up the boundary
            curr_level = Levels[i]
            #Find the starting point for the boundary
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
            #Trace the boundary
            first_node = (start_i, start_j)
            #find first curr node after first step
            curr_node = np.array([start_i, start_j])
            first = True
            curr_ori = np.array([0,1])
            max_i = len(curr_level)
            max_j = len(curr_level[0])
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
            #Convert GridMap into Binary Image
            BI = np.zeros((max_i,max_j))
            for p in range(max_i):
                for q in range(max_j):
                    if curr_level[p][q] == "#":
                        BI[p][q] = 1
            #Find any obstacles inside the boundary
            BI = np.uint8(BI)
            # Perform the operation
            nb_components, output, _, _ = cv2.connectedComponentsWithStats(BI,connectivity=4)
            obs_list = []
            if nb_components > 2:
                for r in range(2,nb_components):
                    curr_obs = []
                    for p in range(max_i):
                        for q in range(max_j):
                            if output[p][q] == r:
                                curr_obs.append([p,q])
                    obs_list.append(curr_obs)
            obstacles = []
            #setting up Outlines to fill later
            #Now find the outlines of the obstalces
            #Find the starting point for the boundary
            min_x = int(sys.maxsize)
            max_y = 0
            for coord in curr_outline:
                if coord[0] < min_x:
                    min_x = coord[0]
                if coord[1] > max_y:
                    max_y = coord[1]

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
            #print("Level", i + 1, "Complete")
            min_y = int(sys.maxsize)
            max_x = 0
            for coord in curr_outline:
                if coord[1] < min_y:
                    min_y = coord[1]
                if coord[0] > max_x:
                    max_x = coord[0]
            #find vehicle, disk and goal positions
            goal_pos_xy = []
            initial_vehicle_pos_xy = []
            initial_disk_pos_xy = []
            for j in range (max_i):
                for k in range(max_j):
                    if (curr_level[j][k] == '.'):
                        #goal
                        goal_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted
                    elif (curr_level[j][k] == '@'):
                        #vehicle
                        initial_vehicle_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted
                    elif (curr_level[j][k] == '$'):
                        #disk
                        initial_disk_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted
                    elif (curr_level[j][k] == '*'):
                        #disk on top of goal
                        initial_disk_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted
                        goal_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted
                    elif (curr_level[j][k] == '+'):
                        #vehicle on top of goal
                        initial_vehicle_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted
                        goal_pos_xy.append(np.array([k-min_x-0.5,max_y+0.5-j])) #converted

            curr_map = Map(i+1, min_x, min_y, max_x, max_y, 1, curr_outline, obstacles, 0.45, 0.45, goal_pos_xy, initial_vehicle_pos_xy, initial_disk_pos_xy)
            self._test_maps.append(curr_map)

        #for i in range(50): #len(test_maps)
        #    print("Test Map: ",test_maps[i].number)
        #    print("Range: [",test_maps[i].min_x, "->", test_maps[i].max_x, ", ",test_maps[i].min_y, "->", test_maps[i].max_y, "]")
        #    print("Grid size: ",test_maps[i].grid_size)
        #    print("Disk radius: ",test_maps[i].disk_radius)
        #    print("Vehicle radius: ",test_maps[i].vehicle_radius)
        #    print("Boundary")
        #    for j in range(len(test_maps[i].boundary)):
        #        print(test_maps[i].boundary[j])
        #    print("Obstacles")
        #    for j in range(len(test_maps[i].obstacles)):
        #        print("Obstacle", j+1)
        #        for k in range(len(test_maps[i].obstacles[j])):
        #            print(test_maps[i].obstacles[j][k])
        #    print("Vehicle Position: ", test_maps[i].initial_vehicle_pos_xy)
        #    print("Goal Positions")
        #    for j in range(len(test_maps[i].goal_pos_xy)):
        #        print(test_maps[i].goal_pos_xy[j])
        #    print("Disk Positions")
        #    for j in range(len(test_maps[i].initial_disk_pos_xy)):
        #        print(test_maps[i].initial_disk_pos_xy[j])

    @property
    def test_maps(self):
        return self._test_maps