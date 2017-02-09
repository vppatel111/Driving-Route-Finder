import csv
import sys
import queue
from math import sqrt


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
    todolist = queue.deque([start])
    reached = {start: (start, 0, [start])}
    while todolist:
        v = todolist.popleft()
        for w in graph.neighbours(v):  # for each neighbhour to v
            total_cost = cost(v, w) + reached[v][1]
            if w not in reached:
                reached[w] = (v, total_cost, reached[v][2]+[w])
                todolist.append(w)  # find neighbours to w
            elif reached[w][1] > total_cost:  # elif better path cost
                reached[w] = (v, total_cost, reached[v][2]+[w])
    if dest not in reached:
        return []
    return reached[dest][2]


# NOTE: Define a cost function within the read_graph functions
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
                # print(row)

                if (row[0] == 'V'):
                    g.add_vertex(int(row[1]))
                    verticesInfo[int(row[1])] = (int(float(row[2]) * 100000),
                                                 int(float(row[3]) * 100000))
                    # print("created vertex", row[1])

                if (row[0] == 'E'):
                    edge = (int(row[1]), int(row[2]))
                    g.add_edge(edge)
                    edgesInfo[edge] = (row[3])
                    # print("created edge", row[1], row[2])

    return (g, verticesInfo, edgesInfo)


def cost_distance(u, v):
    ''' Computes and returns the straight-line distance between the two
        vertices u and v.

        Args:
            u, v:   The ids for two vertices that are the start and
                    end of a valid edge in the graph.
        Returns:
            numeric value: the distance between the two vertices.
    '''
    # print(u, v)
    (x1, y1) = verticesInfo[u]
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
    v1 = (abs(v[0]), abs(v[1]))
    v2 = (abs(u[0]), abs(u[1]))
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

    vertex = (latitude, longitude)
    # print("vertex", vertex)
    return min(vertices, key=lambda vertex1=vertices.get:
               vertexDist(vertices[vertex1], vertex))


file_name = "edmonton-roads-2.0.1.txt"
verticesInfo = {}  # {vertex: (latitude, longitude)}
edgesInfo = {}  # {(vertex1, vertex2): (street_name)}

g, verticesInfo, edgesInfo = read_graph(file_name, verticesInfo, edgesInfo)
# print(verticesInfo)
# print(edgesInfo)
# print(g.vertices())

if __name__ == "__main__":
    outputBuffer = []  # Buffer for outputting to std.out

    for line in sys.stdin:
        line = line.split()
        # print(line)

        # Skip if line is empty
        if line == "":
            continue

        # Arduino requests next data in buffer.
        if (line[0] == "A") and (outputBuffer):
            print(outputBuffer.pop(0))

        # Arduino sends request for map.
        if line[0] == "R":
            startLat = int(float(line[1]))
            startLon = int(float(line[2]))
            # print("start lat/lon", startLat, startLon)
            endLat = int(float(line[3]))
            endLon = int(float(line[4]))
            # print("end lat/lon", endLat, endLon)
            start = findVertex(startLat, startLon, verticesInfo)
            end = findVertex(endLat, endLon, verticesInfo)
            shortest_path = least_cost_path(g, start, end, cost_distance)
            outputBuffer.append(len(shortest_path))
            for waypoint in shortest_path:
                (outputLat, outputLon) = verticesInfo[waypoint]
                outputBuffer.append("W " + str(outputLat) + " " + str(outputLon))
            outputBuffer.append("E")
            # print("start and end", start, end)

        # print(startLat, startLon, endLat, endLon)
