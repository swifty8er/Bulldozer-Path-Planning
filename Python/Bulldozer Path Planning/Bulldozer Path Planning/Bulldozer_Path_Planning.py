import queue
import time
from matplotlib import pyplot as plt
import imageio
from xlrd import open_workbook
from xlutils.copy import copy
import sim
import math
import numpy as np

from Maps import Maps
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
from TranspositionTable import TranspositionTable
from MapState import MapState
from Graph import Graph
from BasicGeometry import BasicGeometry


NUM_OF_BITS = 32
TRANS_TABLE_SIZE = 15
file_path = "C:/Users/cBrak/Documents/UNSW2020/thesis/Bulldozer-Path-Planning/Python/Bulldozer Path Planning/Bulldozer Path Planning/Microban Levels.txt"
myMap = Maps(file_path)


fig1, ax1 = plt.subplots(1, 1)

# Main loop over all the test maps
#for map in myMap.test_maps:
num = 0
#mapNums = list(range(1,36))+list(range(38,77))+list(range(78,83))+list(range(84,93))+list(range(94,97))
#mapNums = list(range(88,93))+list(range(94,97))
#mapNums = list(range(89,93))+list(range(94,97))+list(range(100,105))+[107,110,111,113,115,116]+list(range(118,122))
mapNums = list(range(84,89))
#for mm in range(num,num+10):
#for mm in range(num,num+1):
for mm in mapNums:
    map = myMap.test_maps[mm-1]
    print("Test Map", map.number)
    curr_state = MapState(map)
    trans_table = TranspositionTable(curr_state.num_of_nodes, NUM_OF_BITS, TRANS_TABLE_SIZE)
    pq = queue.PriorityQueue()
    currPQState = PQState(curr_state.GetHeuristicValue(),curr_state,0)
    pq.put(currPQState)
    start_time = time.time()

    #fig, ax = plt.subplots(1, 1)
    #map.plotMap(ax, True)

    while ((pq.empty() == False) and (curr_state.isFinishState() == False) and (time.time() - start_time <= 3600)):
        currPQState = pq.get()
        curr_state = currPQState.state #using property decorator/function
        #print("Current Node")
        #curr_node.printNode()
        if (trans_table.isVisited(curr_state, True) == False):
            #decisions = curr_state.findReachablePushPoints(curr_node.vehicle_path, curr_node.disk_path)
            decisions = curr_state.findReachablePushPoints() #push required variables into the state
            # decisions is a list of map states
            #print("\nList of Decisions")
            for decision in decisions:
                    status = trans_table.addToTable(decision) #check if state already explored
                    #decision.printNode()
                    if status == "E" or status == "R":
                        pq.put(PQState(decision.GetHeuristicValue()+decision.g,decision,decision.g))

            curr_state.resetGraphs()

    
    #Open the excel file
    read_book = open_workbook("Results/TestResults.xls")
    work_book = copy(read_book)
    read_sheet = read_book.sheet_by_index(0)
    work_sheet = work_book.get_sheet(0)

    if curr_state.isFinishState() == True:
        #print("Solution")
        #print("Time:", time.time() - start_time)
        #print("Vehicle Postion:", curr_node.vehicle_pos)
        #print("Disk Positions:")
        #for disk in curr_node.disk_poses:
        #    print(disk)
        #print("Vehicle Path:")
        #for point in curr_node.vehicle_path:
        #    print(point)
        #print("Disk Paths:")
        #i = 1
        #for path in curr_node.disk_path:
        #    print("Disk", i, "'s Path")
        #    for point in path:
        #        print(point)
        #    i += 1

        #Store results in an excel file      
        #if (read_sheet.cell_value(map.number+4, 4) == ''):
        #Add info that map was solved
        work_sheet.write(map.number+4, 4,'True')

        #add time solve in to work sheet
        counter = 0
       
        while(read_sheet.nrows>=map.number+4 and read_sheet.ncols-2>=counter+6 and read_sheet.cell_value(map.number+4, counter + 6) != ''):
            counter += 1
        work_sheet.write(map.number+4, counter + 6, time.time() - start_time)

        work_book.save("Results/TestResults.xls")

        #Save results as a gif
        kwargs_write = {'fps':1.0, 'quantizer':'nq'}
        file_path = 'Gifs/Map ' + str(map.number) +'.gif'
        imageio.mimsave(file_path, curr_state.plotSolution(curr_node.vehicle_path, curr_node.disk_path), fps=1)

    else:
        #Store results in an excel file      
        #if (read_sheet.cell_value(map.number+4, 4) == ''):
            #Add info that map was unsolved or timed out
        if (time.time() - start_time > 3600):
            work_sheet.write(map.number+4, 4,'Timed Out')
        else:
            work_sheet.write(map.number+4, 4,'Unsolved')

        #add time solve in to work sheet
        if(read_sheet.cell_value(map.number+4, 6) == ''):
            work_sheet.write(map.number+4, 6, time.time() - start_time)

        work_book.save("Results/TestResults.xls")
        

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