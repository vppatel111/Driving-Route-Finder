'''Last change: 9:10am Jan 28, 2017. breadth_first_search bugfix.
'''

import queue


def breadth_first_search(g, v):
    '''Discovers all vertices in graph g reachable from vertex v
    and returns the search graph. Paths on the search graph
    are guaranteed to follow shortest paths from v.

    Args:
        g (graph): Graph to search in.
        v (vertex of g): Where the search starts from.

    Returns:
        dict: Dictionary whose keys are nodes u discovered with value being
        the node from where the node described by the key was discovered.
        By definition, v is discovered from v.
    '''
    todolist = queue.deque([v])  # todolist also stores "from where"
    reached = {v: v}
    while todolist:
        u = todolist.popleft()
        for w in g.neighbours(u):
            if w not in reached:
                reached[w] = u  # w has just been discovered
                todolist.append(w)
    return reached


if __name__ == "__main__":
    # this is only run when the script is not imported,
    # but RuntimeError
    import adjacencygraph
    # normally you put imports at the beginning of the file,
    # except when you need some module only locally for e.g. testing purposes.
    g = {'a': 'cbh', 'b': 'cd', 'd': 'cl', 'e': 'abj', 'f': 'ek', 'g': 'df',
         'h': 'e', 'i': 'hj', 'j': 'fl', 'k': 'g', 'l': 'k'}
    graph = adjacencygraph.AdjacencyGraph()
    for (v, e) in g.items():
        for u in e:
            graph.add_edge((v, u), autocreation=True)
    reached = breadth_first_search(graph, 'a')
    print(sorted(list(reached.keys())))

    graph = adjacencygraph.UndirectedAdjacencyGraph()
    for (v, e) in g.items():
        for u in e:
            graph.add_edge((v, u), autocreation=True)
    reached = breadth_first_search(graph, 'a')
    print(sorted(list(reached.keys())))
