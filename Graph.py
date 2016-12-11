#python 3.x code
import io
from collections import deque
from enum import Enum

class GraphNode:
    class Color(Enum):
        White = 0
        Grey  = 1
        Black = 2
    '''
    To define GraphNode type, which represents node in graph
    '''
    def __init__(self, _name):
        self._outNeighbors = [] #out neighbors
        self._outWeights = [] # weight to out neighbors. weights of edges
        self._name = _name
        self._id = -1
        self._visited  = False
        self._color = GraphNode.Color.White

    def print(self):
        print("Node: {0}]".format(self._name), end = " ")
        self.printNeighbors()

    def getName(self):
        return self._name

    def printNeighbors(self):
        print("[Neightors ", end = "")
        index = 0
        for n in self._outNeighbors:
            print(" ({0}, {1} )".format(n._name, self._outWeights[index]), end = '')
            index += 1
        print(']')

    def addNeighbor(self, to, weight = 1):
        self._outNeighbors.append(to)
        self._outWeights.append(weight)

    def setVisited(self, val):
        if(isinstance(val, bool)):
            self._visited = val
    def getVisited(self):
        return self._visited
    def getNeighbors(self):
        return self._outNeighbors
    def getWeight(self, index):
        if(index >= len(self._outWeights)): raise AssertionError("Out of index to get out neighbor weight") 
        return self._outWeights[index]

    def resetColor(self):
        self._color = GraphNode.Color.White
    def setColorFinish(self):
        self._color = GraphNode.Color.Black
    def setColorIntermediate(self):
        self._color = GraphNode.Color.Grey
    
    def dfs(self, printTrace = False, topologicalSortResult = None):
        if(self._color == GraphNode.Color.White):
            self._color = GraphNode.Color.Grey
            if(printTrace): print("[Node d] {}".format(self._name), end = " " )
            for n in self.getNeighbors():
                n.dfs( printTrace, topologicalSortResult)
            self._color = GraphNode.Color.Black
            if(printTrace): print("[Node f] {}".format(self._name), end = " " )
            if(isinstance(topologicalSortResult, list)): topologicalSortResult.append(self)

class Graph:
    '''
        to define graph
    '''
    def __init__(self, isDirected = False):
        self._nodes = {}
        self._numOfEdges = 0
        self._isDirected = isDirected
    def addNode(self, node):
        nodeName = node.getName()
        if(nodeName not in self._nodes):
            self._nodes[nodeName] = node
        else:
            print("Node with name [{0}] has in graph".format(nodeName))

    def getNode(self, nodeName):
        node = None
        if(isinstance(nodeName, str)):
            if( nodeName in self._nodes ):
                node = self._nodes[nodeName]
        return node

    def addEdge(self, frm, to, weight = 1 ):
        '''
            To add an edge to the graph
        '''
        if not isinstance(frm, GraphNode):
            print("frm node is not a type of GraphNode", file=sys.stderr)
            return
        if not isinstance(to, GraphNode):
            print("to node is not a type of GraphNode", file=sys.stderr)
            return
        if(not self._isDirected): 
            to.addNeighbor(frm, weight)
            self._numOfEdges += 1
        frm.addNeighbor(to, weight)
        self._numOfEdges += 1
    def numOfNodes(self):
        return len(self._nodes.values())

    def print(self):
        '''
        To print out graph node by node, for each node, print out adjacency list
        '''
        print("Graph is Directed {}".format(self._isDirected))
        for value in self._nodes.values():
            value.print()
    def resetVisitedFlags(self):
        for value in self._nodes.values():
            value.setVisited(False)
    
    def dfs(self, printTrace = False, topologicalSortResult = None):
        '''
        Do Depth first search on the whole graph
        '''
        #intialization
        for n in self._nodes.values():
            n.resetColor()
        for n in self._nodes.values():
            if(n._color == GraphNode.Color.White):
                n.dfs(printTrace, topologicalSortResult)

    def topologicalSort(self):
        '''
        Do topoligical sort on the whole graph by using dfs
        '''
        sortedResult = []
        self.dfs(True, sortedResult)
    
    def dfsWithStack(node, printTrace = False):
        if( not isinstance(node, GraphNode)):
            return

        stack = [node]
        while(len(stack) >0):
            n = stack.pop()
            if(n.getVisited()):
                continue
            n.setVisited(True)
            if(printTrace): print( "node d[{}]".format(n.getName()), end = " ")
            for nn in n.getNeighbors():
                if(not nn.getVisited()):
                    stack.append(nn)
            if(printTrace): print( "node f[{}]".format(n.getName()), end = " ")

    def bfs(node, printTrace = False):
        queue = deque([node])
        while(len(queue) >0):
            n = queue.popleft()
            if(n.getVisited()):
                continue
            n.setVisited(True)
            if(printTrace):
                print("node[{}]".format(n.getName()), end = ' ')
            for nn in n.getNeighbors():
                if(not nn.getVisited()):
                    queue.append(nn)
    def dijkstra(g, frm, to):
        '''
        To find shortest paths from start to all other vertices using dijkstra algorithm
        Note that it only works with none-negative-weight graph
        '''
        if(frm == None or to == None): return
        last = frm

        nodeHistory = {}
        for n in g._nodes.values():
            nodeHistory[n] = [-1, False];
        results = []

        minWeight  = -1
        nextCandidate = None
        while(last != to):
            if( nextCandidate != None and minWeight == -1):
                print("Cannot find further path anymore, dijkstra shortest path stop here")
                break;#cannot find further path any more
            index  = 0
            for n in last.getNeighbors():
                weight = last.getWeight(index)
                index += 1
                
                history = nodeHistory[n] 
                if(history[1] == True): continue
                if(history[0] < 0) : history[0] = weight
                else:
                    newWeight = minWeight + weight
                    if(history[0] > newWeight): history[0] = newWeight;
            #select v with min weight;
            minWeight = -1
            nextCandidate  = None
            for n in nodeHistory.keys():
                history = nodeHistory[n]
                if(history[1] == True):
                    continue
                if(minWeight < 0):
                    minWeight = nodeHistory[n][0]
                    nextCandidate = n
                elif( minWeight > weight):
                    minWeight = weight
                    nextCandidate = n;
            
            last = nextCandidate 
            nodeHistory[last][1] = True
            results.append(last)
        for n in results:
            print (n.getName(), end = ' ')
            
    def load(graphInStr):
        '''
            To load graph from string.
            example of definition:
            Graph
            True
            Nodes
            5,a,b,c,d,e
            Edges
            5,a b,a c,a d,a e,b c,b d,b
        '''
        g = None
        if graphInStr :
            g = None
            with io.StringIO(graphInStr) as f:
                tag = ''; frmNodeStr = ''; toNodeStr = ''
                for line in f:
                    line = line.strip()
                    if(line == 'Graph'):
                        tag = 'graph'
                        #print("tag of graph")
                    if(line == 'Nodes'):
                        tag = 'node'
                    elif( line == "Edges"):
                        tag = 'edge'
                    elif( tag == 'graph'):
                        #expect to have True or False to tell if the graph is directed
                        isDirected = line.upper()
                        g = Graph(isDirected == 'TRUE')
                        #print('graph initialized')
                    elif(tag == 'node'):#to load nodes
                        if(g == None): raise AssertionError("Graph is not initialized yet when trying to load nodes")
                        for nodeName in line.split(','):
                            nodeName = nodeName.strip()
                            if(nodeName):
                                node = GraphNode(nodeName)
                                g.addNode(node)
                    elif(tag == 'edge'):#to load edges
                        if(g == None): raise AssertionError("Graph is not initialized yet when trying to load edges")
                        if(g.numOfNodes() == 0): raise AssertionError("Graph has no node, but trying to load edges")
                        for edgeStr in line.split(' '):
                            edgeStr = edgeStr.strip()
                            nodes = edgeStr.split(',')
                            if(len(nodes) > 2):
                                frmNodeStr = nodes[0].strip()
                                toNodeStr = nodes[1].strip()
                                weight = int(nodes[2].strip())
                                if(frmNodeStr and toNodeStr and weight):
                                    #print('trying to add edge [{}]-[{}]'.format(frmNodeStr, toNodeStr))
                                    nodeFrom = g.getNode(frmNodeStr)
                                    nodeTo = g.getNode(toNodeStr)
                                    if( nodeFrom and nodeTo):
                                        #print('adding edge:[{}]-[{}]'.format(frmNodeStr, toNodeStr))
                                        g.addEdge(nodeFrom, nodeTo, weight)

        return g

def testCaseCreateSimpleGraph():
    print("========Start test[Create Simple Graph]====")
    g = Graph()
    neast = GraphNode("East")
    g.addNode(neast)
    westNode = GraphNode("west")
    g.addNode(westNode)
    g.addEdge(neast, westNode)
    g.print()
    print("========End test[Create Simple Graph]====")
def testCaseLoadGraph():
    print('======== Start test[Load Graph from string]==')
    graphStr = '''
    Graph
    True
    Nodes
    5,a,b,c,d,e
    Edges
    a,b,1 a,c,1 a,d,1 a,e,1 b,c,1 b,d,1 b,e,1
    '''
    print("Graph \n {}".format(graphStr))
    g = Graph.load(graphStr)
    if( g ):
        g.print()
    else:
        print("cannot load graph", file= sys.stderr)
        print("Graph str for references: {0}".format(graphStr))
    print('======== End test[Load Graph from string]==')
def testCaseDFS():
    print("Graph test: DFS")
    graphStr = '''
    Graph
    False
    Nodes
    0,1,2,3,4,5
    Edges
    0,1,1 0,2,1 0,3,1 1,3,1 2,3,1 2,4,1 3,4,1 4,5,1
    '''
    g = Graph.load(graphStr)
    g.print()
    if ( g):
        g.resetVisitedFlags()
        node = g.getNode('0')
        print("\nStart of DFS test")
        g.dfs(True)
    print("\n===================End of DFS test========================")

def testCaseDFSWithStack():
    print("Graph test: DFSWithStack")
    graphStr = '''
    Graph
    False
    Nodes
    0,1,2,3,4,5
    Edges
    0,1,1 0,2,1 0,3,1 1,3,1 2,3,1 2,4,1 3,4,1 4,5,1
    '''
    g = Graph.load(graphStr)
    if ( g):
        g.print()
        g.resetVisitedFlags()
        node = g.getNode('0')
        print("Start of DFSWithStack test")
        Graph.dfsWithStack( node, True)
    print("\n==================End of DFSwithStack test============================")

def testCaseBFS():
    print("Graph test: bfs")
    graphStr = '''
    Graph
    False
    Nodes
    ton,1,2,3,4,van
    Edges
    ton,1,2 ton,2,3 ton,3,3 1,3,1 2,3,3 2,4,2 3,4,2 4,van,2
    '''
    print("Graph str {0}".format(graphStr))
    g = Graph.load(graphStr)
    g.print()
    if ( g):
        g.resetVisitedFlags()
        node = g.getNode('ton')
        print("Start of BFS test")
        Graph.bfs( node, True)
    print("\n==========================End of bfs test==========================")

def testDijkstra():
    print("Graph: Dijkstra test")
    graphStr = '''
    Graph
    True
    Nodes
    A,B,C,D,E,F,G,H
    Edges
    A,B,20 A,D,80 A,G,90 B,F,10 C,H,20 C,F,50 C,D,10 D,C,10 D,G,20 
    E,B,50 E,G,30 F,C,10 F,D,40 G,A,20
    '''
    g = Graph.load(graphStr)
    if(g):
        g.print()
        frm = g.getNode('A')
        to  = g.getNode('H')
        if frm and to :
            print("Shortest Path from [{0}] to [{1}]".format(frm._name, to._name))
            Graph.dijkstra(g, frm, to)
    
    print("\n============End of Dijkstra test=======================")

if __name__ == "__main__" :
    #testCaseCreateSimpleGraph()
    #testCaseLoadGraph()
    testCaseDFS()
    testCaseDFSWithStack()
    #testCaseBFS()
    testDijkstra()
