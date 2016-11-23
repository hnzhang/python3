#python 3.x code
import io
from collections import deque

class GraphNode:
    '''
    To define GraphNode type, which represents node in graph
    '''
    def __init__(self, _name):
        self._neighbors = []
        self._value = 0
        self._weight = 1
        self._name = _name
        self._visited  = False

    def print(self):
        print("Node value: {0}]".format(self._name), end = " ")
        self.printNeighbors()

    def getName(self):
        return self._name

    def printNeighbors(self):
        print("[Neightors] ", end = "")
        for n in self._neighbors:
            print(" {0} ".format(n._name), end = '')
        print()

    def addNeighbor(self, to):
        self._neighbors.append(to)

    def setVisited(self, val):
        if(isinstance(val, bool)):
            self._visited = val
    def getVisited(self):
        return self._visited
    def getNeighbors(self):
        return self._neighbors

class Graph:
    '''
        to define graph
    '''
    def __init__(self):
        self._nodes = {}
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

    def print(self):
        '''
        To print out graph node by node, for each node, print out adjacency list
        '''
        for item in self._nodes.items():
            print("Node Value: ", end = " ")
            item[1].print()
    def resetVisitedFlags(self):
        for value in self._nodes.values():
            value.setVisited(False)

    def dfs( node, printTrace = False):
        #self.ressetVisitedFlags(False)
        if( not isinstance(node, GraphNode)):
            return
        if(node.getVisited() == True):
            return

        if(printTrace):
            print( "node[{}]".format(node.getName()), end = " ")
        node.setVisited(True)
        for n in node.getNeighbors():
            Graph.dfs(n, printTrace) 

    def dfsWithStack(node, printTrace = False):
        if( not isinstance(node, GraphNode)):
            return

        stack = [node]
        while(len(stack) >0):
            n = stack.pop()
            if(n.getVisited()):
                continue
            n.setVisited(True)
            if(printTrace):
                print( "node[{}]".format(n.getName()), end = " ")
            for nn in n.getNeighbors():
                if(not nn.getVisited()):
                    stack.append(nn)

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

    def load(graphInStr):
        '''
            To load graph from string.
            example of definition:
            Nodes
            5,a,b,c,d,e
            Edges
            5,a b,a c,a d,a e,b c,b d,b
        '''
        g = None
        if graphInStr :
            g = Graph()
            with io.StringIO(graphInStr) as f:
                tag = ''; frmNodeStr = ''; toNodeStr = ''
                for line in f:
                    line = line.strip()
                    if(line == 'Nodes'):
                        tag = 'node'
                    elif( line == "Edges"):
                        tag = 'edge'
                    elif(tag == 'node'):
                        for nodeName in line.split(','):
                            nodeName = nodeName.strip()
                            if(nodeName):
                                node = GraphNode(nodeName)
                                g.addNode(node)
                    elif(tag == 'edge'):
                        for edgeStr in line.split(' '):
                            edgeStr = edgeStr.strip()
                            nodes = edgeStr.split(',')
                            if(len(nodes) > 1):
                                frmNodeStr = nodes[0].strip()
                                toNodeStr = nodes[1].strip()
                                if(frmNodeStr and toNodeStr):
                                    print('trying to add edge [{}]-[{}]'.format(frmNodeStr, toNodeStr))
                                    nodeFrom = g.getNode(frmNodeStr)
                                    nodeTo = g.getNode(toNodeStr)
                                    if( nodeFrom and nodeTo):
                                        print('adding edge:[{}]-[{}]'.format(frmNodeStr, toNodeStr))
                                        g.addEdge(nodeFrom, nodeTo)

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
    Nodes
    5,a,b,c,d,e
    Edges
    a,b a,c a,d a,e b,c b,d b,e
    '''
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
    Nodes
    0,1,2,3,4,5
    Edges
    0,1 0,2 0,3 1,3 2,3 2,4 3,4 4,5
    '''
    g = Graph.load(graphStr)
    #g.print()
    if ( g):
        g.resetVisitedFlags()
        node = g.getNode('0')
        Graph.dfs( node, True)
    print("\nEnd of DFS test")

def testCaseDFSWithStack():
    print("Graph test: DFSWithStack")
    graphStr = '''
    Nodes
    0,1,2,3,4,5
    Edges
    0,1 0,2 0,3 1,3 2,3 2,4 3,4 4,5
    '''
    g = Graph.load(graphStr)
    #g.print()
    if ( g):
        g.resetVisitedFlags()
        node = g.getNode('0')
        Graph.dfsWithStack( node, True)
    print("\nEnd of DFSwithStack test")

def testCaseBFS():
    print("Graph test: bfs")
    graphStr = '''
    Nodes
    0,1,2,3,4,5
    Edges
    0,1 0,2 0,3 1,3 2,3 2,4 3,4 4,5
    '''
    g = Graph.load(graphStr)
    #g.print()
    if ( g):
        g.resetVisitedFlags()
        node = g.getNode('0')
        Graph.bfs( node, True)
    print("\nEnd of bfs test")


if __name__ == "__main__" :
    testCaseCreateSimpleGraph()
    testCaseLoadGraph()
    testCaseDFS()
    testCaseDFSWithStack()
    testCaseBFS()

