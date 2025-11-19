# to generate coarse path for TurtleBot3

import numpy as np

def generate_linear_waypoints(start, goal, num_waypoints):
    """
    Generates linear waypoints between start and goal positions.

    :param start: Tuple (x, y) for the start position.
    :param goal: Tuple (x, y) for the goal position.
    :param num_waypoints: Number of waypoints to generate.
    :return: List of waypoints as tuples.
    """
    waypoints = []
    for i in range(num_waypoints + 1):
        t = i / num_waypoints
        x = (1 - t) * start[0] + t * goal[0]
        y = (1 - t) * start[1] + t * goal[1]
        waypoints.append((x, y))
    return waypoints