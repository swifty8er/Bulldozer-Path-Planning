from Maps import Maps
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
import time
from matplotlib import pyplot as plt

myMap = Maps()

fig1, ax1 = plt.subplots(1, 1)

for map in myMap.test_maps:
    vg = VisibilityGraph(map)
    pg = PushabilityGraph(map, vg)
    #map.plotMap(2,ax1)
    #vg.plotGraph(ax1, 2, "blue")
    #pg.plotGraph(ax1, 2, "red")

    #input("Press Enter to continue...")
    ##time.sleep(2)
    #plt.cla()