#python 3.x code

class GraphNode:
    '''
    To define GraphNode type, which represents node in graph
    '''
    def __init__(self, _name):
        self._neighbors = []
        self._value = 0
        self._weight = 1
        self._name = _name

    def print(self):
        print("Node value: {0}]".format(self._name), end = " ")
        self.printNeighbors()

    def printNeighbors(self):
        print("[Neightors] ", end = "")
        for n in self._neighbors:
            print(" {0} ".format(n._name), end = '')
        print()

    def addNeighbor(self, to):
        self._neighbors.append(to)


class Graph:
    '''
        to define graph
    '''
    def __init__(self):
        self._nodes = {}
    def addNode(self, node):
        nodeid = id(node)
        if(nodeid not in self._nodes):
            self._nodes[nodeid] = node
        else:
            print("Node with ID has in graph")
    def addEdge(self, frm, to ):
        '''
            To add an edge to the graph
        '''
        if not isinstance(frm, GraphNode):
            print("frm node is not a type of GraphNode", file=sys.stderr)
            return
        if not isinstance(to, GraphNode):
            print("to node is not a type of GraphNode", file=sys.stderr)
            return
        to.addNeighbor(frm)
        frm.addNeighbor(to)

    def printGraph(self):
        '''
        To print out graph node by node, for each node, print out adjacency list
        '''
        for item in self._nodes.items():
            print("Node Value: ", end = " ")
            item[1].print()
    def load(graphInStr):
        '''
            To load graph from string.
            example of definition:
            Nodes
            5
            a
            b
            c
            d
            e
            Edges
            7
            a
            b
            a
            c
            a
            d
            a
            e
            b
            c
            b
            d
            b
            e
        '''
        if graphInStr :
            f = io.StringIO(graphInStr)
            g = Graph()
            f.readline()


def testCase():
    print("========Start test====")
    g = Graph()
    neast = GraphNode("East")
    g.addNode(neast)
    westNode = GraphNode("west")
    g.addNode(westNode)
    g.addEdge(neast, westNode)
    g.printGraph()

if __name__ == "__main__" :
    testCase()

