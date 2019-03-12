#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import numpy as np
from check_collision import check_line_collision

def k_nearest(milestones, point, k):
    """ Gets the k nearest milestones to a point """

    d = np.zeros((milestones.shape[0], 2))

    for i in range(milestones.shape[0]):
        d[i][0] = i
        d[i][1] = np.linalg.norm(point - milestones[i][:])
    d = d[d[:, 1].argsort()]
    return d[0:k, :]

def get_all_k_nearest_neighbours(milestones, k):
    """ Gets a list of neighbours for all the milestones """

    neighbours = []

    for i in range(len(milestones)):
        neighbours.append(k_nearest(milestones, milestones[i], k))

    return neighbours

def remove_invalid_neighbours(milestones, neighbours, occupancy_grid):
    """ Removes invalid neighbours from neighbour list due to collisions """

    for i in range(len(milestones)):
        valid = np.zeros(len(neighbours[i])).astype(bool)
        for j, neighbour in enumerate(neighbours[i]):
            valid[j] = not check_line_collision(occupancy_grid, milestones[i], milestones[int(neighbour[0])])
        neighbours[i] = neighbours[i][valid]

    return neighbours

def generate_distance_graph(milestones, neighbours):
    """ Generates a distance graph of milestones based on the provided neighbours """

    num_milestones = milestones.shape[0]
    graph = np.ones((num_milestones, num_milestones))*np.inf

    for i in range(len(milestones)):
        for neighbour in neighbours[i]:
            idx = int(neighbour[0])
            dist = neighbour[1]
            if (dist < graph[i, idx] or dist < graph[idx, i]):
                graph[i, idx] = dist
                graph[idx, i] = dist

    return graph
