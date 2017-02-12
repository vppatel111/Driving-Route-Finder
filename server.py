"""
Server for Driving Route Finder
by: Vishal Patel & Rizwan Qureshi - EB2

Description: This program is the server portion of the Driving Route Finder.
If run by using "python3 server.py" in the terminal will cause the program to
enter server mode where the user to request a path can input:
"R currentLatitude currentLongitude, destinationLatitude destinationLongitude",
the server will first find the closest vertices to the start and end points
and will respond first with "N #_of_nodes" and a buffered output of each
waypoint to the desired location that is printed when the user inputs "A"
followed by an enter, when the server runs out of waypoints it will print "E".

Additionally this program can be imported using server.py to use the
least_cost_path, read_graph, cost_distance, vertexDist, and findVertex
functions.

Accessories: None
Wiring Instructions: None
Additional Functionality: The program contains the minheap.py code developed
in class.

To run as server: 1. Use the $ python3 server.py command
2. To send a request for the shortest path to a destination type in stdin:
"R currentLatitude currentLongitude, destinationLatitude destinationLongitude"
3. Once a request has been sent the server will output "N #_of_nodes"
4. The rest of the output will be buffered and sent when the user inputs "A",
followed by an enter.
5. When the server is finished printing the waypoints it will output an "E",
followed by an enter.

To use the least_cost_path, read_graph, cost_distance, vertexDist, and
findVertex functions as well as the minheap class in another program:
1. Use import server to import entire server module.
Or
Use import server <function_name> to import individual components of the server
module.
"""
import csv
import sys
from math import sqrt


class MinHeap:

    def __init__(self):
        self._array = []

    def add(self, key, value):
        self._array.append((key, value))
        self.fix_heap_up(len(self._array)-1)

    def pop_min(self):
        if not self._array:
            raise RuntimeError("Attempt to call pop_min on empty heap")
        retval = self._array[0]
        self._array[0] = self._array[-1]
        del self._array[-1]
        if self._array:
            self.fix_heap_down(0)
        return retval

    def fix_heap_up(self, i):
        if self.isroot(i):
            return
        p = self.parent(i)
        if self._array[i][0] < self._array[p][0]:
            self.swap(i, p)
            self.fix_heap_up(p)

    def swap(self, i, j):
        self._array[i], self._array[j] = \
            self._array[j], self._array[i]

    def isroot(self, i):
        return i == 0

    def isleaf(self, i):
        return self.lchild(i) >= len(self._array)

    def lchild(self, i):
        return 2*i+1

    def rchild(self, i):
        return 2*i+2

    def parent(self, i):
        return (i-1)//2

    def min_child_index(self, i):
        l = self.lchild(i)
        r = self.rchild(i)
        retval = l
        if r < len(self._array) and self._array[r][0] < self._array[l][0]:
            retval = r
        return retval

    def isempty(self):
        return len(self._array) == 0

    def length(self):
        return len(self._array)

    def fix_heap_down(self, i):
        if self.isleaf(i):
            return

        j = self.min_child_index(i)
        if self._array[i][0] > self._array[j][0]:
            self.swap(i, j)
            self.fix_heap_down(j)

    # Some stnadard collection interfaces

    # So the len() function will work.
    def __len__(self):
        return len(self._array)

    # Iterator
    def __iter__(self):
        return iter(self._array)

    def __next__(self):
        return (self._array).__next__


def least_cost_path(graph, start, dest, cost):
    """Find and return a least cost path in graph from start
        vertex to dest vertex.

    Efficiency: If E is the number of edges, the run-time is
                O( E log(E) ).

    Args:
        graph:  The digraph defining the edges between the
                vertices.
        start:  The vertex where the path starts. It is assumed
                that start is a vertex of graph.
        dest:   The vertex where the path ends. It is assumed
                that start is a vertex of graph.
        cost:   A function, taking the two vertices of an edge as
                parameters and returning the cost of the edge. For its
                interface, see the definition of cost_distance.

    Returns:
        list:   A potentially empty list (if no path can be found) of
                the vertices in the graph. If there was a path, the first
                vertex is always start, the last is always dest in the list.
                Any two consecutive vertices correspond to some
                edge in graph.

    Other information:
        todolist:   Stores the vertices we must go through to check for cost.
                    As vertices are checked and popped out, the list will add
                    the neighbours of the popped vertex if it hasn't already
                    been checked.
        reached:    Stores the information of all vertices it has reached in
                    one component of the graph. The information is stored in a
                    tuple as follows:
                    (a, b, c)
                    a = The previous edge element
                    b = The total_cost to reach the vertex from the start
                    c = The ordered path from the start to vertex w in listform
    """
    todolist = MinHeap()
    todolist.add(0, start)
    reached = {start: (0, [start])}
    while todolist:
        v = todolist.pop_min()
        if v[1] == dest:
            break
        for w in graph.neighbours(v[1]):  # For each neighbour to v
            total_cost = cost(v[1], w) + v[0]  # Keep track of cost so far
            if w not in reached:
                reached[w] = (total_cost, reached[v[1]][1]+[w])
                todolist.add(total_cost, w)  # Find more neighbours
            elif reached[w][0] > total_cost:  # elif better path cost
                reached[w] = (total_cost, reached[v[1]][1]+[w])
    if dest not in reached:
        return []
    return reached[dest][1]


def read_graph(file_name, verticesInfo, edgesInfo):
    ''' Reads from a .csv file and creates an UndirectedAdjacencyGraph object

    Args:
        file_name (string): The file to be read from.
        verticesInfo (dict): Contains info: vertex: (latitude, longitude)}
        edgesInfo (dict): Contains info: {(vertex1, vertex2): (street_name)}
    Returns:
        g: Created AdjacencyGraph object
    '''
    import adjacencygraph

    g = adjacencygraph.AdjacencyGraph()

    with open(file_name) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row:
                # break up row into a list
                row = list(row)

                if (row[0] == 'V'):
                    g.add_vertex(int(row[1]))  # Create vertex
                    verticesInfo[int(row[1])] = (int(float(row[2]) * 100000),
                                                 int(float(row[3]) * 100000))

                if (row[0] == 'E'):
                    edge = (int(row[1]), int(row[2]))  # Create edge
                    g.add_edge(edge)
                    edgesInfo[edge] = (row[3])  # Store extra edge info

    return (g, verticesInfo, edgesInfo)


def cost_distance(u, v):
    ''' Computes and returns the straight-line distance between the two
        vertices u and v.

        Args:
            u, v:   The ids for two vertices that are the start and
                    end of a valid edge in the graph.
        Returns:
            (numbericValue): the distance between the two vertices.
    '''
    # print(u, v)
    (x1, y1) = verticesInfo[u]  # Splits vertices into lat/lon components
    (x2, y2) = verticesInfo[v]
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))


# Built upon the concept introduced by cost_distance
def vertexDist(v, u):
    ''' Computes and returns the straight-line distance between a vertex v and
        a point u.

    Args:
        v, u (tuple): (latitude, longitude)
    Returns:
        dist = distance of path
    '''
    v1 = (v[0], v[1])
    v2 = (u[0], u[1])
    pathDist = ((v1[1]-v2[1])*(v1[1]-v2[1])+(v1[0]-v2[0])*(v1[0]-v2[0]))
    return pathDist


# Further testing required
def findVertex(latitude, longitude, vertices):
    ''' Returns closes vertex to given latitude and longitude

    Args:
        latitude (int), longitude (int): latitude and longitude of vertex
        verticesInfo (dict): Contains info: vertex: (latitude, longitude)}
    Returns:
        v = closest vertex
    '''

    v2 = (latitude, longitude)
    # print("vertex", vertex)
    return min(vertices, key=lambda v1=vertices.get:
               vertexDist(vertices[v1], v2))


file_name = "edmonton-roads-2.0.1.txt"
verticesInfo = {}  # {vertex: (latitude, longitude)}
edgesInfo = {}  # {(vertex1, vertex2): (street_name)}

g, verticesInfo, edgesInfo = read_graph(file_name, verticesInfo, edgesInfo)

if __name__ == "__main__":
    outputBuffer = []  # Buffer for outputting to std.out

    for line in sys.stdin:
        line = line.split()

        # Skip if line is empty
        if line == "":
            continue

        # Arduino requests next data in buffer.
        if (line[0] == "A") and (outputBuffer):
            print(outputBuffer.pop(0))

        # Arduino sends request for map.
        if line[0] == "R":
            # Read in individual pieces of data.
            startLat = int(float(line[1]))
            startLon = int(float(line[2]))
            endLat = int(float(line[3]))
            endLon = int(float(line[4]))
            # Find start and finish
            start = findVertex(startLat, startLon, verticesInfo)
            end = findVertex(endLat, endLon, verticesInfo)
            # Calculate shortest path
            shortest_path = least_cost_path(g, start, end, cost_distance)
            print("N " + str(len(shortest_path)))  # Print number of waypoint
            for waypoint in shortest_path:  # Fill up output buffer.
                (outputLat, outputLon) = verticesInfo[waypoint]
                outputBuffer.append("W " + str(outputLat) + " " +
                                    str(outputLon))
            if len(shortest_path) > 0:  # Accounts for an empty path
                outputBuffer.append("E")
