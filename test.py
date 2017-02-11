'''
This file is strictly for the sake of testing
Run using the following command in the command line (make sure you are in the right directory!):
            python3 test.py
'''
from adjacencygraph import AdjacencyGraph as Graph
from time import time
from dijkstra_algorithm import least_cost_path
#from cost_distance import cost_distance
from server import read_graph

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
'''
test_case:
1: v1 = 29577354, v2 = 1455345886
2: v1 = 30198552, v2 = 225181634
'''
fileName = "edmonton-roads-2.0.1.txt"
verticesInfo = {}
edgesInfo = {}
t = time()
#g, verticesInfo, edgesInfo = read_graph(fileName, verticesInfo, edgesInfo)
print(time()-t)
t = time()
#shortest_path = least_cost_path(g, 30198552, 1455345886, cost_distance)
s = {1,2,3,4,5,6}
l = [(1,2), (1,3), (1,6), (2,1),
(2,3), (2,4), (3,1), (3,2), (3,4), (3,6), (4,2), (4,3),
(4,5), (5,4), (5,6), (6,1), (6,3), (6,5)]
weights = {(1,2): 7, (1,3):9, (1,6):14, (2,1):7, (2,3):10,
(2,4):15, (3,1):9, (3,2):10, (3,4):11, (3,6):2,
(4,2):15, (4,3):11, (4,5):6, (5,4):6, (5,6):9, (6,1):14,
(6,3):2, (6,5):9}
cost = lambda e: weights.get(e, float("inf"))
print(least_cost_path(graph, 1, 5, cost))
print(time()-t)
print(shortest_path)
