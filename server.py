import csv
import sys


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
                    g.add_vertex(row[1])
                    verticesInfo[row[1]] = (int(float(row[2]) * 100000),
                                            int(float(row[3]) * 100000))
                    # print("created vertex", row[1])

                if (row[0] == 'E'):
                    edge = (row[1], row[2])
                    g.add_edge(edge)
                    edgesInfo[edge] = row[3]
                    # print("created edge", row[1], row[2])

    return (g, verticesInfo, edgesInfo)


# Built upon the concept introduced by pathDist
def pathDist(v, u):
    ''' Computes and returns the straight-line distance between the two vertices
        v and u.

    Args:
        v, u (tuple): (latitude, longitude)
    Returns:
        dist = distance of path
    '''
    v1 = (abs(v[0]), abs(v[1]))
    v2 = (abs(u[0]), abs(u[1]))
    pathDist = ((v1[1]-v2[1])*(v1[1]-v2[1])+(v1[0]-v2[0])*(v1[0]-v2[0]))
    print("pathDist", pathDist, v1, v2)
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
               pathDist(vertices[vertex1], vertex))


file_name = "edmonton-roads-2.0.1.txt"
verticesInfo = {}  # {vertex: (latitude, longitude)}
edgesInfo = {}  # {(vertex1, vertex2): (street_name)}

g, verticesInfo, edgesInfo = read_graph(file_name, verticesInfo, edgesInfo)

# print(verticesInfo)
# print(edgesInfo)
# print(g.vertices())

outputBuffer = []  # Buffer for outputting to std.out

# NOTE: Test code. REMOVE LATER
# supermin = min(verticesInfo, key=verticesInfo.get)
# print(supermin)

print(verticesInfo)

for line in sys.stdin:
    line = line.split()
    # print(line)

    # Skip if line is empty
    if line == "":
        continue

    # Arduino requests next data in buffer.
    if line[0] == "A":
        print(outputBuffer.pop(0))

    # Arduino sends request for map.
    if line[0] == "R":
        startLat = int(float(line[1]))
        startLon = int(float(line[2]))
        print("start lat/lon", startLat, startLon)
        endLat = int(float(line[3]))
        endLon = int(float(line[4]))
        print("end lat/lon", endLat, endLon)
        start = findVertex(startLat, startLon, verticesInfo)
        end = findVertex(endLat, endLon, verticesInfo)
        # path = least_cost_path(g, start, end, _)
        # outputBuffer.append(len(path))
        print("start and end", start, end)

    # print(startLat, startLon, endLat, endLon)
