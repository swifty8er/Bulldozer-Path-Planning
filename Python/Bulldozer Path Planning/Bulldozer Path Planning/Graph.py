from matplotlib import pyplot as plt

from BasicGeometry import BasicGeometry

class Graph():
    #A simple directed graph class

    def __init__(self, nodes):

        self._nodes = nodes.deepcopy()
        self._graph = dict()
        for i in range(len(nodes)):
            self._graph[i] = dict()

    #Check to see if node i has any connections. Returns true if connections are found
    def hasConnections(self, i):
        return i in self._graph.keys()

    #Check to see if an edge in the graph exists. Returns true if the edge exists
    def validEdge(self, i, j):
        return j in self._graph[i].keys()

    def validNode(self, i):
        return i in self._graph.keys()

    #Returns the weight of a particular edge in the graph. If the edge doesn't exist then -1 is returned (weights are assumed to always be positive)
    def edgeWeight(self, i, j):
        weight = -1
        if self.validEdge(i,j) == True:
            weight = self._graph[i][j]

        return weight

    #Find all the direct connecting nodes to a certain node in the graph. Returns an empty list if there are no connections
    def connectingNodes(self, i):
        nodes = []
        if self.hasConnections(i) == True:
            nodes = self._graph[i].keys()

        return nodes

    #Returns a list of nodes that have connections to other nodes.
    def nodesWithConnections(self):
        conn_nodes = []
        for node in self._graph.keys():
            if len(self._graph[node].keys()) > 0:
                conn_nodes.append(node)
        return conn_nodes

    #Add an edge to the graph
    def addEdge(self, i, j, weight):
        self._graph[i][j] = weight

    #Set the weight of an edge in the graph. An error print message appears if the edge doesn't exist
    def setEdgeWeight(self, i, j, weight):
        if self.validEdge(i, j):
            self._graph[i][j] = weight
        if self.validEdge(j, i):
            self._graph[j][i] = weight
    
    #Remove an edge from the graph. Returns the removed edges
    def removeEdge(self, i, j):
        removed_edges = []
        if self.validEdge(i, j):
            del self._graph[i][j]
            removed_edges.append([i,j])
        if self.validEdge(j, i):
            del self._graph[j][i]
            removed_edges.append([j,i])

        return removed_edges

    #Remove all the edges from a node. Returns the removed edges
    def removeAllNodesEdges(self, i):
        removed_edges = []
        if self.validNode(i):
            for node in self._graph[i].keys():
                del self._graph[node][i]
                removed_edges.append([node,i])
                removed_edges.append([i, node])
            self._graph[i] = dict()

        return removed_edges


    @property
    def nodes(self):
        return self._nodes
   


    def plotGraph(self, ax, line_size=2, colour="blue"):
        for i in range(len(self._nodes)):
            for j in range(len(self._nodes)):
                if self.validEdge(i, j)  == True:
                    ax.plot([self._nodes[i][0],self._nodes[j][0]],[self._nodes[i][1],self._nodes[j][1]], color=colour, linewidth=line_size)
                    #plt.draw()
                    #plt.pause(0.001)
                    #plt.show(block=False)
                    #print("cool")
        plt.draw()
        plt.pause(0.001)
        plt.show(block=False)

