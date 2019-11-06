from Maps import Maps
from VisibilityGraph import VisibilityGraph
import time
from matplotlib import pyplot as plt

myMap = Maps()

fig1, ax1 = plt.subplots(1, 1)

for map in myMap.test_maps:
    vg = VisibilityGraph(map)
    #map.plotMap(2,ax1)
    #vg.plotGraph(map.nodes,2,ax1)
    #time.sleep(2)
    #plt.cla()