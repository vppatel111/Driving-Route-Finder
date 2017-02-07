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
                    c = The ordered path from the start to vertex w in list form
    """
    todolist = queue.deque([start])
    reached = {start: (start, 0, [start])}
    while todolist:
        v = todolist.popleft()
        for w in g.neighbours(v): # for each neighbhour to v
            total_cost = cost(v,w) + reached[v][1]
            if w not in reached:
                reached[w] = (v, total_cost, reached[v][2].append(w))
                todolist.append(w) # find neighbours to w
            elif reached[w][1] > total_cost: #elif better path cost
                reached[w] = (v, total_cost, reached[v][2].append(w))
    if dest not in reached:
        return []
    return reached[dest][2]
