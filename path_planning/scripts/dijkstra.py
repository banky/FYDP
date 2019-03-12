#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import numpy as np
import itertools

def meta_dijkstra(graph_array, waypoints):
    """ Computes shortest path if we have to stop through waypoints """

    num_waypoints = len(waypoints)
    meta_graph = np.full((num_waypoints, num_waypoints), np.inf)
    combinations = list(itertools.combinations(waypoints, 2))
    paths = {}
    distances = {}
    for pair in combinations:
        path, dist = dijkstra(graph_array, pair[0], pair[1])
        paths[str(list(pair))] = path
        paths[str(list(reversed(pair)))] = list(reversed(path))
        distances[str(list(pair))] = dist
        distances[str(list(reversed(pair)))] = dist
    all_meta_paths = list(itertools.permutations(waypoints))
    total_path_lenth = np.inf
    final_meta_path = []
    final_total_path = []
    for meta_path in all_meta_paths:
        dist = 0
        for i in range(len(meta_path) - 1):
            dist += distances[str(list([meta_path[i], meta_path[i+1]]))]
        if dist < total_path_lenth:
            total_path_lenth = dist
            final_meta_path = meta_path
    for i in range(len(final_meta_path) - 1):
        sub_path = paths[str(list([final_meta_path[i],final_meta_path[i+1]]))]
        if (i == 0):
            final_total_path = [sub_path[0]]
        final_total_path  = final_total_path + sub_path[1:]
    return final_total_path, total_path_lenth

def dijkstra(graph_array, start_idx, end_idx, debug=False):
    """ Computes absolute shortest path """

    # Make unvistited list
    num_nodes = graph_array.shape[0]
    unvisited_nodes = np.ones(num_nodes)
    unvisited_nodes_dist = np.full((num_nodes), np.inf)
    tentative_dist = np.full((num_nodes), np.inf)
    tentative_dist[start_idx] = 0

    path = [[] for i in range(num_nodes)]

    curr_node_idx = start_idx
    curr_node_dist = 0
    count = 1
    while(curr_node_idx != end_idx and ~np.isinf(curr_node_dist)):
        if debug:
            print("Current node: " + str(curr_node_idx) + " Dist: " + str(curr_node_dist))
            print("Path to current: ")
            print(path[curr_node_idx])
            print("Conected, unvistited nodes: ")
        for i in range(num_nodes):
            if (unvisited_nodes[i] == 1
                    and ~np.isinf(graph_array[curr_node_idx][i])
                    and i != curr_node_idx):
                dist_neighbor_thru_curr = curr_node_dist + \
                                         graph_array[curr_node_idx][i]
                if debug:
                    print(i)
                if (dist_neighbor_thru_curr < tentative_dist[i]):
                    tentative_dist[i] = dist_neighbor_thru_curr
                    unvisited_nodes_dist[i] = dist_neighbor_thru_curr
                    path[i] = list(path[curr_node_idx])
                    path[i].append(curr_node_idx)


        unvisited_nodes[curr_node_idx] = 0
        unvisited_nodes_dist[curr_node_idx] = np.inf
        curr_node_idx = np.argmin(unvisited_nodes_dist)
        curr_node_dist = tentative_dist[curr_node_idx]
        if debug:
            print("tentative_dist")
            print(tentative_dist)

            print("unvisited_nodes_dist")
            print(unvisited_nodes_dist)

            print
        count += 1
    if(np.isinf(tentative_dist[end_idx])):
        return([], np.inf)
    path[end_idx].append(end_idx)
    return(path[end_idx], tentative_dist[end_idx])

if __name__ == '__main__':

    def add_edge(graph, idx1, idx2, val):
        graph[idx1, idx2] = val
        graph[idx2, idx1] = val
        return graph
    # Test
    graph1 = np.full((6,6), np.inf)
    graph1 = add_edge(graph1,0,1,2)
    graph1 = add_edge(graph1,1,2,3)
    graph1 = add_edge(graph1,1,3,2)
    graph1 = add_edge(graph1,2,4,1)
    graph1 = add_edge(graph1,3,4,4)
    print(graph1)
    print
    exp = ([0, 1, 2, 4], 6.0)
    # ans = dijkstra(graph1, 0, 4)
    # print("exp: " + str(exp))
    # print("ans: " + str(ans))
    # print(ans == exp)
    # print
    print("META")
    print(meta_dijkstra(graph1, [0, 4]))
    print(meta_dijkstra(graph1, [0,3,4]))