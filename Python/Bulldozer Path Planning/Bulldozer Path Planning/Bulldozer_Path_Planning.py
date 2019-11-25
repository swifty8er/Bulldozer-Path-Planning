import queue
import time
from matplotlib import pyplot as plt
import imageio

from Maps import Maps
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
from TranspositionTable import TranspositionTable
from MapState import MapState
from Graph import Graph
from BasicGeometry import BasicGeometry


NUM_OF_BITS = 32
TRANS_TABLE_SIZE = 15

myMap = Maps()

fig1, ax1 = plt.subplots(1, 1)

for map in myMap.test_maps:
    print("Test Map", map.number)
    curr_state = MapState(map)
    trans_table = TranspositionTable(curr_state.num_of_nodes, NUM_OF_BITS, TRANS_TABLE_SIZE)
    pq = queue.PriorityQueue()
    curr_node = curr_state.getCurrentState()
    pq = queue.PriorityQueue()
    pq.put(curr_node)
    start_time = time.time()

    while ((pq.empty() == False) and (curr_state.isFinishState() == False) and (time.time() - start_time <= 3600)):
        curr_node = pq.get()
        if (trans_table.isVisited(curr_node, True) == False):
            curr_state.updateState(curr_node)
            decisions = curr_state.findReachablePushPoints(curr_node.vehicle_path, curr_node.disk_path)
            #print("\nList of Decisions")
            for decision in decisions:
                    status = trans_table.addToTable(decision)
                    #if (2 in decision.disk_poses) and (4 in decision.disk_poses):
                    #    decision._cf -= 0.01
                    #decision.printNode()
                    if status == "E" or status == "R":
                        pq.put(decision)

            curr_state.resetGraphs()

    if curr_state.isFinishState() == True:
        print("Solution")
        print("Time:", time.time() - start_time)
        print("Vehicle Postion:", curr_node.vehicle_pos)
        print("Disk Positions:")
        for disk in curr_node.disk_poses:
            print(disk)
        print("Vehicle Path:")
        for point in curr_node.vehicle_path:
            print(point)
        print("Disk Paths:")
        i = 1
        for path in curr_node.disk_path:
            print("Disk", i, "'s Path")
            for point in path:
                print(point)
            i += 1

        #Store results in an excel file

        #Save results as a gif
        
        kwargs_write = {'fps':1.0, 'quantizer':'nq'}
        file_path = 'Gifs/Map ' + str(map.number) +'.gif'
        imageio.mimsave(file_path, curr_state.plotSolution(curr_node.vehicle_path, curr_node.disk_path), fps=1)
        


