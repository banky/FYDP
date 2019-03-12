#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import numpy as np

def grid_at_pos(occupancy_grid, x, y):
    """ Return true if point is on obstacle """
    width = occupancy_grid.shape[0]
    height = occupancy_grid.shape[1]

    if (x > width or y > height):
        return 1.0

    return occupancy_grid[x][y]


def check_line_collision(occupancy_grid, p1, p2):
    """ Check if any point on line collides with an obstacle.
        This function uses the Bresenham Line algorithm """

    delta_x = float(p2[0] - p1[0])
    delta_y = float(p2[1] - p1[1])
    error = 0.0

    if delta_x == 0 and delta_y == 0:
        return False

    # Swap for steep lines
    if abs(delta_y) > abs(delta_x):
        x = p1[0]
        delta_err = abs(delta_x / delta_y)
        for y in np.linspace(p1[1], p2[1], abs(p1[1] - p2[1])+1):
            if (grid_at_pos(occupancy_grid, int(x), int(y))):
                return True
            error = error + delta_err
            if error >= 0.5:
                x += np.sign(delta_x)
                error = error - 1.0
    else:
        y = p1[1]
        delta_err = abs(delta_y / delta_x)
        for x in np.linspace(p1[0], p2[0], abs(p1[0] - p2[0])+1):
            if (grid_at_pos(occupancy_grid, int(x), int(y))):
                return True
            error = error + delta_err
            if error >= 0.5:
                y += np.sign(delta_y)
                error = error - 1.0

    return False
