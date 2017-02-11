'''
This file is strictly for the sake of testing
Run using the following command in the command line (make sure you are in the right directory!):
            python3 test.py
'''
import adjacencygraph
from time import time
#from dijkstra_algorithm import least_cost_path
#from cost_distance import cost_distance
import server

def cost_distance(u, v):
    ''' Computes and returns the straight-line distance between the two
        vertices u and v.

        Args:
            u, v:   The ids for two vertices that are the start and
                    end of a valid edge in the graph.
        Returns:
            numeric value: the distance between the two vertices.
    '''
    global verticesInfo
    (x1, y1) = verticesInfo[u]
    (x2, y2) = verticesInfo[v]
    return (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)

def print_coord(verticesInfo, key):
    print("{} {}".format(verticesInfo[key][0],verticesInfo[key][1]))

fileName = "edmonton-roads-2.0.1.txt"
verticesInfo = {}
edgesInfo = {}
t = time()
g, verticesInfo, edgesInfo = server.read_graph(fileName, verticesInfo, edgesInfo)
print(time()-t)
t = time()
shortest_path = server.least_cost_path(g, 314088878, 629239028, cost_distance)
print(time()-t)
print(shortest_path)
