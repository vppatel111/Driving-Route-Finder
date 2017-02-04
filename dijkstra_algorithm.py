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
    """
