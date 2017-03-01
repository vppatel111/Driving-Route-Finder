"""
Server for Driving Route Finder
by: Vishal Patel & Rizwan Qureshi - EB2

Description: This program is the server portion of the Driving Route Finder.
If run by using "python3 server.py" in the terminal will cause the program to
enter server mode in which the server will first attempt to connect to the
Arduino client and begin communications if the Arduino is found. The client can
then request a path from the serial-port by sending:
"R currentLatitude currentLongitude, destinationLatitude destinationLongitude",
the server will first find the closest vertices to the start and end points
and will respond first with "N #_of_nodes" and a buffered output of each
waypoint to the desired location that is printed when the client acknowledgeds
with "A\n" when the server runs out of waypoints it will output "E\n".

Additionally this program can be imported using server.py to use the
least_cost_path, read_graph, cost_distance, vertexDist, and findVertex
functions.

Accessories: Joystick, LCD Screen
Wiring Instructions: Can be found in circuit-wiring.txt
Additional Functionality: The program contains the minheap.py code developed
in class.

To set up server: 1. Navigate to the Driving-Router-Finder directory.
2. Use the $ python3 server.py command.

To set up client: 1. Navigate to the Driving-Router-Finder directory.
2. Ensure the Arduino is connected to the PC.
3. Use make upload to upload the client code to the Arduin.

To use the program: - We can use the joystick to navigate around the screen.
- We can use the buttons to zoom in and out of the map.
- To get the shortest path between 2 points:
1. Select two points by moving the joystick and click the joystick at the start
and end points.
2. Within 1-10 seconds a path from the start and destination will appear.

To use the least_cost_path, read_graph, cost_distance, vertexDist, and
findVertex functions as well as the minheap class in another program:
1. Use import server to import entire server module.
Or
Use import server <function_name> to import individual components of the server
module.

Additional information: - Changes made to files: server.py, client.cpp,
serial_handling.cpp, serial_handling.h
"""
import csv
# import sys
from cs_message import *
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
                    been checked. The vertices to check are prioritized by min
                    value.
        reached:    Stores the information of all vertices it has reached in
                    one component of the graph. The information is stored in a
                    tuple as follows:
                    (a, b)
                    a = The total_cost to reach the vertex from the start
                    b = The ordered path from the start to vertex w in listform
    """
    todolist = MinHeap()
    todolist.add(0, start)
    reached = {start: (0, [start])}
    while todolist:  # list vertices we go through, with a priority queue
        v = todolist.pop_min()
        if v[1] == dest:  # once we have found our destination don't continue
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


def getAck(serial_in):
    ''' Waits for a acknowledgement from the client and returns a false in the
        case of a timeout and true otherwise.

    Args:
        serial_in - serial object used to obtain data from the serial port
    Returns:
        bool state - true if acknowledgement found otherwise returns false
    '''

    try:  # Waits for message if timeout, we get exception StopIteration.
        msg = receive_msg_from_client(serial_in)
        log_msg(msg)
        if msg[0] == "A":
            return True
        elif msg != "":
            return False
    except StopIteration:  # Timed out
        return False


def getMsg(serial_in):
    ''' Waits for a message from the client and returns the message, in the
        case of a timeout it will return an invalid message ("_").

    Args:
        serial_in - (serial object), used to obtain data from the serial port
    Returns:
        msg - message read from serial port or ("_") if no message found.
    '''

    try:  # Waits for message if timeout, we get exception StopIteration.
        msg = receive_msg_from_client(serial_in)
        # log_msg("Got path:")
        log_msg(msg)
        return msg
    except StopIteration:  # Timed out
        return "_"


def server(serial_in, serial_out):
    ''' Contains a finite state machine in charge of processing inputs and
        sending out data.

    Args:
        serial_in - (serial object), used to obtain data from the serial port
        serial_out - (serial object), used to send data to the serial port
    Returns:
        None
    '''

    while True:
        while True:  # Waits unil valid request from waypoints..
            msg = getMsg(serial_in)
            if msg[0] == "R":  # Is message found valid in context.
                break

        # Assume that it's a properly formatted R message
        coords = msg[2:].split()
        if len(coords) != 4:  # Make sure we got all coords
            continue

        (lat_s, lon_s, lat_e, lon_e) = coords
        # Read in individual pieces of data.
        startLat = int(float(lat_s))
        startLon = int(float(lon_s))
        endLat = int(float(lat_e))
        endLon = int(float(lon_e))

        # Check for zero displacement
        if ((startLat == endLat) and (startLon == endLon)):
            send_msg_to_client(serial_out, "N 0")
            log_msg("Zero displacement")
            continue

        # Find start and finish
        start = findVertex(startLat, startLon, verticesInfo)
        end = findVertex(endLat, endLon, verticesInfo)

        # Calculate shortest path
        shortest_path = least_cost_path(g, start, end, cost_distance)

        # Write the number of waypoints to client
        n = len(shortest_path)
        send_msg_to_client(serial_out, "N {}" .format(n))

        if n > 0:  # If non-zero path

            ack = getAck(serial_in)  # Wait for client acknowledgement
            if not ack:
                log_msg("Timeout: No Ack")
                continue

            for waypoint in shortest_path:  # Begin printing waypoints

                (outputLat, outputLon) = verticesInfo[waypoint]
                send_msg_to_client(serial_out,
                                   "W {} {}" .format(outputLat, outputLon))

                ack = getAck(serial_in)
                if not ack:  # Completely reset if not given ack. (Part 1)
                    log_msg("Timeout: No Ack")
                    break

            if not ack:  # Completely reset if not given ack. (Part 2)
                log_msg("Timeout: No Ack")
                continue

            send_msg_to_client(serial_out, "E")  # Finished printing waypoints
        else:  # Otherwise begin waiting for new request.
            continue


file_name = "edmonton-roads-2.0.1.txt"
verticesInfo = {}  # {vertex: (latitude, longitude)}
edgesInfo = {}  # {(vertex1, vertex2): (street_name)}

g, verticesInfo, edgesInfo = read_graph(file_name, verticesInfo, edgesInfo)

if __name__ == "__main__":
    import textserial

    # Setting to default values for now, use argparse later
    serial_port_name = "/dev/ttyACM0"
    log_msg("Opening serial port: {}".format(serial_port_name))
    baudrate = 9600  # [bit/seconds] 115200 also works

    # Open up the connection, with timeout of 1 second.
    with textserial.TextSerial(
            serial_port_name, baudrate, timeout=1, newline=None) as ser:
            log_msg("Restarting server")
            server(ser, ser)
