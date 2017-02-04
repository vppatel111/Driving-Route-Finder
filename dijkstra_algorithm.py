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
    todolist = queue.deque([start])  # todolist also stores "from where"
    # reached is a dictionary containing the tuple:
    # (other edge element, cost of edge, list of the path to reach key vertex)
    reached = {start: (start, 0, [start])}
    while todolist:
        v = todolist.popleft()
        for w in g.neighbours(v): # for each neighbhour to v
            edge_cost = cost((v,w))
            if w not in reached:
                reached[w] = (v, edge_cost, reached[v][2].append(w))  # w has just been discovered
                todolist.append(w) # find neighbours to w
            elif (e for i, e, l in reached[w]) > edge_cost: #elif better path cost
                reached[w] = (v, edge_cost, reached[v][2].append(w))
    return reached[dest][2]
