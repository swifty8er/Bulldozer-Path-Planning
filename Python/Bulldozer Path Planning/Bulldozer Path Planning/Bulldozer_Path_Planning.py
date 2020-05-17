import queue
import time
from matplotlib import pyplot as plt
import imageio
from xlrd import open_workbook
from xlutils.copy import copy
import sim
import math
import numpy as np
import random

from Maps import Maps
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
from TranspositionTable import TranspositionTable
from MapState import MapState
from Graph import Graph
from RRT import RRT
from RRT import Status
from PQState import PQState
from BasicGeometry import BasicGeometry
from Vehicle import Vehicle


myMap = Maps()

ControlsList = [
    (0.4,45,"FL"),
    (0.4826,37.3,"FL"),
    (0.593,30.37,"FL"),
    (0.7493,24.02,"FL"),
    (0.991,18.16,"FL"),
    (1.405,12.8,"FL"),
    (2.223,8.097,"FL"),
    (4.199,4.286,"FL"),
    ((0.4*math.pi)/4.0,0,"F"),
    (0.4,45,"FR"),
    (0.4826,37.3,"FR"),
    (0.593,30.37,"FR"),
    (0.7493,24.02,"FR"),
    (0.991,18.16,"FR"),
    (1.405,12.8,"FR"),
    (2.223,8.097,"FR"),
    (4.199,4.286,"FR"),
    (0.4,45,"RL"),
    (0.4826,37.3,"RL"),
    (0.593,30.37,"RL"),
    (0.7493,24.02,"RL"),
    (0.991,18.16,"RL"),
    (1.405,12.8,"RL"),
    (2.223,8.097,"RL"),
    (4.199,4.286,"RL"),
    ((0.4*math.pi)/4.0,0,"R"),
    (0.4,45,"RR"),
    (0.4826,37.3,"RR"),
    (0.593,30.37,"RR"),
    (0.7493,24.02,"RR"),
    (0.991,18.16,"RR"),
    (1.405,12.8,"RR"),
    (2.223,8.097,"RR"),
    (4.199,4.286,"RR")]


fig1, ax1 = plt.subplots(1, 1)

# Main loop over all the test maps
#for map in myMap.test_maps:
num = 0
#mapNums = list(range(1,36))+list(range(38,77))+list(range(78,83))+list(range(84,93))+list(range(94,97))
mapNums = [1]
#mapNums = list(range(88,93))+list(range(94,97))
#mapNums = list(range(1,4))
#for mm in range(num,num+10):
#for mm in range(num,num+1):
for mm in mapNums:
    map = myMap.test_maps[mm-1]
    print("Test Map", map.number)
    map.plotStartingMap(ax1)
    plt.draw()
    plt.pause(1)
    plt.show(block=False)
    x_range = map.max_x - map.min_x
    y_range = map.max_y - map.min_y
    num_nodes = int(x_range * y_range * 200)
    starting_xy = map.initial_vehicle_pos_xy
    StartVehiclePos = Vehicle(starting_xy[0],starting_xy[1],90) #change to random heading
    StartingRRT = RRT(map,StartVehiclePos,ControlsList,num_nodes)
    i = 0
    while i < StartingRRT.num_nodes:
        print("i = ",i)
        x_rand = StartingRRT.generateRandomState()
        status = StartingRRT.extend(x_rand)
        if (status == Status.ADVANCED or status == Status.REACHED):
            i+=1


    curr_state = PQState(map,StartVehiclePos,None,map.initial_disk_pos_xy,[],[[] for x in range(len(map.initial_disk_pos_xy))],[False]*len(map.goal_pos_xy),-1,[],StartingRRT,0)
    visitedStates = {}
    pq = queue.PriorityQueue()
    pq.put(curr_state)
    start_time = time.time()

    while not pq.empty():
        curr_state = pq.get()
        plt.cla()
        curr_state.plotState(ax1)
        plt.draw()
        plt.pause(0.1)
        plt.show(block=False)
        if not curr_state.connectToPreviousPose():
            continue
        if curr_state.isFinishState():
            break
        if not curr_state in visitedStates:
            visitedStates[curr_state] = True
            new_states = curr_state.getResultingStates(ax1)
            for state in new_states:
                if not state in visitedStates:
                    pq.put(state)

    if curr_state.isFinishState() == True:
        print("Solved in minutes = ",(time.time() - start_time)/60)
        #Save results as a gif
        kwargs_write = {'fps':25.0, 'quantizer':'nq'}
        file_path = 'ElliottGifs/Map ' + str(map.number) +'.gif'
        imageio.mimsave(file_path, curr_state.plotSolution(), fps=25)
    else:
        print("Failed")
    #curr_state = MapState(map)
    #trans_table = TranspositionTable(curr_state.num_of_nodes, NUM_OF_BITS, TRANS_TABLE_SIZE)
    #pq = queue.PriorityQueue()
    #curr_node = curr_state.getCurrentState()
    #pq = queue.PriorityQueue()
    #pq.put(curr_node)
    #start_time = time.time()

    ##fig, ax = plt.subplots(1, 1)
    ##map.plotMap(ax, True)

    #while ((pq.empty() == False) and (curr_state.isFinishState() == False) and (time.time() - start_time <= 3600)):
    #    curr_node = pq.get()
    #    #print("Current Node")
    #    #curr_node.printNode()
    #    if (trans_table.isVisited(curr_node, True) == False):
    #        curr_state.updateState(curr_node)
    #        decisions = curr_state.findReachablePushPoints(curr_node.vehicle_path, curr_node.disk_path)
    #        #print("\nList of Decisions")
    #        for decision in decisions:
    #                status = trans_table.addToTable(decision)
    #                #decision.printNode()
    #                if status == "E" or status == "R":
    #                    pq.put(decision)

    #        curr_state.resetGraphs()

    
    ##Open the excel file
    #read_book = open_workbook("Results/TestResults.xls")
    #work_book = copy(read_book)
    #read_sheet = read_book.sheet_by_index(0)
    #work_sheet = work_book.get_sheet(0)

    #if curr_state.isFinishState() == True:
    #    #print("Solution")
    #    #print("Time:", time.time() - start_time)
    #    #print("Vehicle Postion:", curr_node.vehicle_pos)
    #    #print("Disk Positions:")
    #    #for disk in curr_node.disk_poses:
    #    #    print(disk)
    #    #print("Vehicle Path:")
    #    #for point in curr_node.vehicle_path:
    #    #    print(point)
    #    #print("Disk Paths:")
    #    #i = 1
    #    #for path in curr_node.disk_path:
    #    #    print("Disk", i, "'s Path")
    #    #    for point in path:
    #    #        print(point)
    #    #    i += 1

    #    #Store results in an excel file      
    #    #if (read_sheet.cell_value(map.number+4, 4) == ''):
    #    #Add info that map was solved
    #    work_sheet.write(map.number+4, 4,'True')

    #    #add time solve in to work sheet
    #    counter = 0
       
    #    while(read_sheet.nrows>=map.number+4 and read_sheet.ncols-2>=counter+6 and read_sheet.cell_value(map.number+4, counter + 6) != ''):
    #        counter += 1
    #    work_sheet.write(map.number+4, counter + 6, time.time() - start_time)

    #    work_book.save("Results/TestResults.xls")

    #    #Save results as a gif
    #    kwargs_write = {'fps':1.0, 'quantizer':'nq'}
    #    file_path = 'Gifs/Map ' + str(map.number) +'.gif'
    #    imageio.mimsave(file_path, curr_state.plotSolution(curr_node.vehicle_path, curr_node.disk_path), fps=1)

    #else:
    #    #Store results in an excel file      
    #    #if (read_sheet.cell_value(map.number+4, 4) == ''):
    #        #Add info that map was unsolved or timed out
    #    if (time.time() - start_time > 3600):
    #        work_sheet.write(map.number+4, 4,'Timed Out')
    #    else:
    #        work_sheet.write(map.number+4, 4,'Unsolved')

    #    #add time solve in to work sheet
    #    if(read_sheet.cell_value(map.number+4, 6) == ''):
    #        work_sheet.write(map.number+4, 6, time.time() - start_time)

    #    work_book.save("Results/TestResults.xls")
        

    #if map.number == 1:
    #    sim.simxFinish(-1) # just in case, close all opened connections
    #    clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    #    if clientID!=-1:
    #        print ('Connected to remote API server')
    #        error1, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    #        error1, first_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    #        error1, first_ori = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    #        print(first_pos)
    #        time.sleep(0.1)
    #        error1, first_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim. simx_opmode_buffer)
    #        error1, first_ori = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
    #        print(first_pos)
    #        error1, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    #        error2, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    #        if error1 != 0 or error2 != 0:
    #            print("object handle not found")
    #        #Fix Error Handling

    #        #List constants
    #        wheel_radius = 0.0975
    #        radius_to_wheel = 0.13
    #        max_velocity = 0.2
    #        max_ang_velocity = 0.2
    #        max_angle = math.pi
    #        k_angle = max_ang_velocity/max_angle
    #        k_speed = max_velocity/max_angle
    #        max_angle_error = 10*(math.pi/180)
    #        max_dist_error = 0.01
    #        diff_x = first_pos[0] - curr_state.getNodeOrPoint(curr_node.vehicle_path[0])[0]
    #        diff_y = first_pos[1] - curr_state.getNodeOrPoint(curr_node.vehicle_path[0])[1]

    #        #Follow Path Generated
    #        for point in curr_node.vehicle_path:
    #            print(point)
    #            dest_x = curr_state.getNodeOrPoint(point)[0] + diff_x
    #            dest_y = curr_state.getNodeOrPoint(point)[1] + diff_y
    #            #while vehicle isn't close to dest point then continue below
    #            error1, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim. simx_opmode_buffer)
    #            curr_x = pos[0]
    #            curr_y = pos[1]
    #            dist_error = math.sqrt((dest_x - curr_x)**2 *(dest_y - curr_y)**2)
    #            while dist_error > max_dist_error:
    #                dest_angle = np.arctan2(dest_y - curr_y,dest_x - curr_x)
    #                error1, curr_angle = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
    #                angle_error = dest_angle - curr_angle[2]
    #                if abs(angle_error) > math.pi:
    #                    if angle_error > 0:
    #                        angle_error =  angle_error - 2*math.pi
    #                    else:
    #                        angle_error = angle_error + 2*math.pi
    #                print("Distance Error:", dist_error, "Angle Error:", angle_error)
    #                if abs(angle_error) > max_angle_error:
    #                    #Feedback loop control to move to that angle
    #                    ang_velocity = k_angle*angle_error #scale down 0-180 to 0-max ang vel (0.2)
    #                    #set velocities
    #                    wheel_ang_velocity = abs(ang_velocity)*(radius_to_wheel/wheel_radius)
    #                    if ang_velocity > 0:
    #                        #turn the right wheel
    #                        error1 = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, -wheel_ang_velocity, sim.simx_opmode_streaming)
    #                        error2 = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, wheel_ang_velocity, sim.simx_opmode_streaming)
    #                    else:
    #                        #turn the left wheel
    #                        error1 = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, wheel_ang_velocity, sim.simx_opmode_streaming)
    #                        error2 = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, -wheel_ang_velocity, sim.simx_opmode_streaming)

    #                else:
    #                    #Feedback loop control to follow that line
    #                    #dist_error = math.sqrt((dest_x - curr_x)**2 *(dest_y - curr_y)**2)
    #                    speed = k_speed*dist_error #0-180 to 0-max vel
    #                    ang_velocity = k_angle*angle_error #scale down 0-180 to 0-max ang vel
    #                    #set velocities
    #                    wheel_velocity = speed/wheel_radius
    #                    wheel_ang_velocity = abs(ang_velocity)*(radius_to_wheel/wheel_radius)
    #                    if ang_velocity > 0:
    #                        #turn the right wheel
    #                        error1 = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, wheel_velocity, sim.simx_opmode_streaming)
    #                        error2 = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, wheel_velocity + wheel_ang_velocity, sim.simx_opmode_streaming)
    #                    else:
    #                        #turn the left wheel
    #                        error1 = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, wheel_velocity + wheel_ang_velocity, sim.simx_opmode_streaming)
    #                        error2 = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, wheel_velocity, sim.simx_opmode_streaming)

    #                dist_error = math.sqrt((dest_x - curr_x)**2 *(dest_y - curr_y)**2)
    #                error1, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim. simx_opmode_buffer)
    #                curr_x = pos[0]
    #                curr_y = pos[1]

    #        #Path Follow Complete
    #        error1 = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
    #        error2 = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)

    #        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    #        sim.simxGetPingTime(clientID)

    #        # Now close the connection to CoppeliaSim:
    #        sim.simxFinish(clientID)
    #    else:
    #        print ('Failed connecting to remote API server')