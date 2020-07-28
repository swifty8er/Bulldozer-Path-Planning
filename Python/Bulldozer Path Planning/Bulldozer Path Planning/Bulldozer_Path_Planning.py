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
import sys

from Maps import Maps
from RRT import RRT
from RRT import Status
from Quadtree import Quadtree
from PQState import PQState
from BasicGeometry import BasicGeometry
from Vehicle import Vehicle

sys.setrecursionlimit(2000)
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

#hMap = myMap.test_maps[5]
#fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
fig1, ax1 = plt.subplots(1, 1)
#hMap.plotHeatmap(ax1)
#plt.draw()
#plt.pause(3)
#plt.show()
#exit(0)
written = False
w1 = False
# Main loop over all the test maps
#for map in myMap.test_maps:
num = 0
#mapNums = list(range(1,36))+list(range(38,77))+list(range(78,83))+list(range(84,93))+list(range(94,97))
mapNums = [1,2,3,4,5,6]
#mapNums = list(range(88,93))+list(range(94,97))
#mapNums = list(range(1,4))
#for mm in range(num,num+10):
#for mm in range(num,num+1):
for mm in mapNums:
    map = myMap.test_maps[mm-1]
    print("Test Map", map.number)
    map.plotStartingMap(ax1)
    ##map.plotStartingMap(ax2)
    ##map.plotStartingMap(ax3)
    ##map.plotStartingMap(ax4)
    plt.draw()
    plt.pause(1)
    plt.show(block=False)
    startTime = time.time()
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
    print("RRT growth complete")
    endTime = time.time() - startTime
    print("RRT built in time = %.2f seconds" % (endTime))
    if not w1:
        file_out = open("TimingData/RRTBuildTimes.txt",'w')
        file_out.write("RRT with %d nodes built in time = %.2f seconds \n" % (StartingRRT.num_nodes,endTime))
        file_out.close()
        w1 = True
    else:
        file_out = open("TimingData/RRTBuildTimes.txt","a+")
        file_out.write("RRT with %d nodes built in time = %.2f seconds \n" % (StartingRRT.num_nodes,endTime))
        file_out.close()
    #StartingRRT.draw(ax1)
    #StartingRRT.draw(ax2)
    #StartingRRT.draw(ax3)
    #StartingRRT.draw(ax4)
    #plt.draw()
    #plt.pause(3)
    #plt.show(block=False)
    StartingQuadtree = Quadtree(map.getCentreState(),None,StartingRRT.computeMaxDistanceBetweenNodes(map.getCentreState()))
    for node in StartingRRT.tree:
        StartingQuadtree.addState(node)

    print("Quadtree grown to size = ",StartingQuadtree.num_states)
    StartingRRT.setQuadtree(StartingQuadtree)

    curr_state = PQState(map,StartVehiclePos,None,map.initial_disk_pos_xy,[],[],[False]*len(map.goal_pos_xy),-1,[],StartingRRT,0)
    visitedStates = {}
    pq = queue.PriorityQueue()
    pq.put(curr_state)
    start_time = time.time()
    globalReachedGoals = [False] * len(map.goal_pos_xy)

    while not pq.empty() and (time.time() - start_time < 3600):
        curr_state = pq.get()
        plt.cla()
        curr_state.plotState(ax1)
        plt.draw()
        plt.pause(0.01)
        plt.show(block=False)
        if curr_state.isFinishState():
            break
        if not curr_state in visitedStates:
            visitedStates[curr_state] = True
            new_states = curr_state.getResultingStates()
            for state in new_states:
                if not state in visitedStates:
                    pq.put(state)

    if curr_state.isFinishState() == True:
        initTime = (time.time() - start_time)/60.0
        print("Solved in initial time %.2f minutes, bezier smoothing path..." % (initTime))
        #curr_state.drawVehiclePath(ax3)
        curr_state.bezierSmoothSolutionPath()
        #curr_state.drawVehiclePath(ax4)
        solveTime  = ( time.time() - start_time ) /60
        print("Done in minutes = ",solveTime)
        #Save results as a gif
        kwargs_write = {'fps':25.0, 'quantizer':'nq'}
        file_path = 'ElliottGifs/Map ' + str(map.number) +'.gif'
        imageio.mimsave(file_path, curr_state.plotSolution(ax1), fps=25)
        pathLength = curr_state.getFinalPathLength()
        if not written:
            file_out = open("TimingData/Results.txt",'w')
            file_out.write("Map %d solved in %.2f seconds with total path length %.2f [A* search completed in %.2f seconds] \n" % (map.number,solveTime*60.0,pathLength,initTime*60.0))
            file_out.close()
            written = True
        else:
            file_out = open("TimingData/Results.txt","a+")
            file_out.write("Map %d solved in %.2f seconds with total path length %.2f [A* search completed in %.2f seconds] \n" % (map.number,solveTime*60.0,pathLength,initTime*60.0))
            file_out.close()
    else:
        print("Failed")
        if not written:
            file_out = open("TimingData/Results.txt",'w')
            file_out.write("Map %d timeout \n" % (map.number))
            file_out.close()
            written = True
        else:
            file_out = open("TimingData/Results.txt","a+")
            file_out.write("Map %d timeout \n" % (map.number))
            file_out.close()
file_out.close()