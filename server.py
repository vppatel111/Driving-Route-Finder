import csv


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
                    verticesInfo[int(row[1])] = (int(float(row[2]) * 1000000),
                                            int(float(row[3]) * 1000000))
                    # print("created vertex", row[1])

                if (row[0] == 'E'):
                    edge = (int(row[1]), int(row[2]))
                    g.add_edge(edge)
                    edgesInfo[edge] = row[3]
                    # print("created edge", row[1], row[2])

    return (g, verticesInfo, edgesInfo)


file_name = "edmonton-roads-2.0.1.txt"
verticesInfo = {}  # {vertex: (latitude, longitude)}
edgesInfo = {}  # {(vertex1, vertex2): (street_name)}

g, verticesInfo, edgesInfo = read_graph(file_name, verticesInfo, edgesInfo)

# print(verticesInfo)
# print(edgesInfo)
# print(g.vertices())
