import time
from matplotlib import pyplot as plt

from Maps import Maps
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
from TranspositionTable import TranspositionTable
from MapState import MapState
from Graph import Graph


NUM_OF_BITS = 32
TRANS_TABLE_SIZE = 15

myMap = Maps()

fig1, ax1 = plt.subplots(1, 1)

for map in myMap.test_maps:
    curr_state = MapState(map)
    trans_table = TranspositionTable(curr_state.total_num_nodes, NUM_OF_BITS, TRANS_TABLE_SIZE)

    curr_state.updateGraphs()
    curr_state.map.plotMap(2,ax1)
    curr_state.vg.plotGraph(ax1, 2, "blue")
    curr_state.pg.plotGraph(ax1, 2, "red")

    input("Press Enter to continue...")
    #time.sleep(2)
    plt.cla()