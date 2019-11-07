from matplotlib import pyplot as plt

class Graph():
    #A simple directed graph class

    def __init__(self, nodes):
        self._nodes = nodes
        self._graph = dict()
        for i in range(len(nodes)):
            self._graph[i] = dict()


    def has_connections(self, i):
        return i in self._graph.keys()

    def valid_edge(self, i, j):
        return j in self._graph[i].keys()

    def edge_weight(self, i, j):
        weight = -1
        if self.valid_edge(i,j) == True:
            weight = self._graph[i][j]

        return weight

    def connecting_nodes(self, i):
        nodes = []
        if self.has_connections(i) == True:
            nodes = self._graph[i].keys()

        return nodes

    def nodes_with_connections(self):
        return self._graph.keys()

    def plotGraph(self, ax, line_size=2, colour="blue"):
        for i in range(len(self._nodes)):
            for j in range(len(self._nodes)):
                if (j in self._graph[i].keys())  == True:
                    ax.plot([self._nodes[i][0],self._nodes[j][0]],[self._nodes[i][1],self._nodes[j][1]], color=colour, linewidth=line_size)
        plt.draw()
        plt.pause(0.001)
        plt.show(block=False)