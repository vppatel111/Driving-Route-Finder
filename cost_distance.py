def cost_distance(u, v):
    ''' Computes and returns the straight-line distance between the two
        vertices u and v.

        Args:
            u, v:   The ids for two vertices that are the start and
                    end of a valid edge in the graph.
        Returns:
            numeric value: the distance between the two vertices.
    '''
    x1, y1  = verticesInfo.getItem(u) 
    verticesInfo(v) = (x2, y2)
    return (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)
