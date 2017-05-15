class Graph(object):

    def __init__(self, graphDict=None):
        if graphDict == None:
            graphDict = {}
        self.__graphDict = graphDict

    def vertices(self):
        return list(self.__graphDict.keys())

    def edges(self):
        return self.__generateEdges()

    def vertexChildren(self, vertex):
        return self.__graphDict[vertex];

    def vertexParents(self, vertex):
        p = [];
        for v in self.vertices():
            if vertex in self.vertexChildren(v) and vertex not in p:
                p.append(v);
        return p;

    def vertexNeighbours(self, initialVertex):
        n = [];

        for vertex in self.vertices():
            if initialVertex in self.vertexChildren(vertex) and vertex not in n:
                n.append(vertex);

        for child in self.vertexChildren(initialVertex):
            if child not in n:
                n.append(child);

        return n;

    def addVertex(self, vertex):
        if vertex not in self.__graphDict:
            self.__graphDict[vertex] = []

    def addEdge(self, edge):
        edge = set(edge)
        vertex1 = edge.pop()
        if edge:
            vertex2 = edge.pop()
        else:
            vertex2 = vertex1
        if vertex1 in self.__graphDict:
            self.__graphDict[vertex1].append(vertex2)
        else:
            self.__graphDict[vertex1] = [vertex2]

    def __generateEdges(self):
        edges = []
        for vertex in self.__graphDict:
            for neighbour in self.__graphDict[vertex]:
                if {neighbour, vertex} not in edges:
                    edges.append({vertex, neighbour})
        return edges

    def __str__(self):
        res = "vertices: "
        for k in self.__graphDict:
            res += str(k) + " "
        res += "\nedges: "
        for edge in self.__generateEdges():
            res += str(edge) + " "
        return res

    def findIsolatedVertices(self):
        graph = self.__graphDict
        isolated = []
        for vertex in graph:
            print(isolated, vertex)
            if not graph[vertex]:
                isolated += [vertex]
        return isolated

    def findPath(self, startVertex, endVertex, path=[]):
        graph = self.__graphDict
        path = path + [startVertex]
        if startVertex == endVertex:
            return path
        if startVertex not in graph:
            return None
        for vertex in graph[startVertex]:
            if vertex not in path:
                extended_path = self.findPath(vertex, 
                                               endVertex, 
                                               path)
                if extended_path: 
                    return extended_path
        return None
    

    def findAllPaths(self, startVertex, endVertex, path=[]):
        graph = self.__graphDict 
        path = path + [startVertex]
        if startVertex == endVertex:
            return [path]
        if startVertex not in graph:
            return []
        paths = []
        for vertex in graph[startVertex]:
            if vertex not in path:
                extended_paths = self.findAllPaths(vertex, 
                                                     endVertex, 
                                                     path)
                for p in extended_paths: 
                    paths.append(p)
        return paths

    def isConnected(self, 
                     verticesFound = None, 
                     startVertex=None):
        if verticesFound is None:
            verticesFound = set()
        gdict = self.__graphDict        
        vertices = gdict.keys() 
        if not startVertex:
            startVertex = vertices[0]
        verticesFound.add(startVertex)
        if len(verticesFound) != len(vertices):
            for vertex in gdict[startVertex]:
                if vertex not in verticesFound:
                    if self.isConnected(verticesFound, vertex):
                        return True
        else:
            return True
        return False

    def vertexDegree(self, vertex):
        adjacentVertices =  self.__graphDict[vertex]
        degree = len(adjacentVertices) + adjacentVertices.count(vertex)
        return degree

    def degreeSequence(self):
        seq = []
        for vertex in self.__graphDict:
            seq.append(self.vertexDegree(vertex))
        seq.sort(reverse=True)
        return tuple(seq)

    @staticmethod
    def isDegreeSequence(sequence):
        return all( x>=y for x, y in zip(sequence, sequence[1:]))
  

    def delta(self):
        min = 100000000
        for vertex in self.__graphDict:
            vertexDegree = self.vertexDegree(vertex)
            if vertexDegree < min:
                min = vertexDegree
        return min
        
    def Delta(self):
        max = 0
        for vertex in self.__graphDict:
            vertexDegree = self.vertexDegree(vertex)
            if vertexDegree > max:
                max = vertexDegree
        return max

    def density(self):
        g = self.__graphDict
        V = len(g.keys())
        E = len(self.edges())
        return 2.0 * E / (V *(V - 1))

    def diameter(self):
        v = self.vertices() 
        pairs = [ (v[i],v[j]) for i in range(len(v)-1) for j in range(i+1, len(v))]
        smallestPaths = []
        for (s,e) in pairs:
            paths = self.findAllPaths(s,e)
            smallest = sorted(paths, key=len)[0]
            smallestPaths.append(smallest)

        smallestPaths.sort(key=len)
        diameter = len(smallestPaths[-1])
        return diameter

    @staticmethod
    def erdoesGallai(dsequence):
        if sum(dsequence) % 2:
            return False
        if Graph.isDegreeSequence(dsequence):
            for k in range(1,len(dsequence) + 1):
                left = sum(dsequence[:k])
                right =  k * (k-1) + sum([min(x,k) for x in dsequence[k:]])
                if left > right:
                    return False
        else:
            return False
        return True
