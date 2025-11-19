# to generate coarse path for TurtleBot3 usin a* algorithm

# src/turtlebot3_planner/turtlebot3_planner/path_generator.py

import numpy as np
import heapq

def heuristic(a, b):
    """Euclidean distance heuristic for A*"""
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(occupancy_grid, start, goal, allow_diagonal=True):
    """
    Simple A* implementation on a 2D occupancy grid.

    Args:
        occupancy_grid (np.ndarray): 2D grid, 0=free, 1=obstacle
        start (tuple): (i, j) start indices
        goal (tuple): (i, j) goal indices
        allow_diagonal (bool): allow diagonal movement

    Returns:
        path (list of tuples): list of grid indices from start to goal
    """
    neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
    if allow_diagonal:
        neighbors += [(-1,-1),(-1,1),(1,-1),(1,1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        close_set.add(current)
        for i,j in neighbors:
            neighbor = (current[0]+i, current[1]+j)
            if 0 <= neighbor[0] < occupancy_grid.shape[0] and 0 <= neighbor[1] < occupancy_grid.shape[1]:
                if occupancy_grid[neighbor[0]][neighbor[1]] == 1:
                    continue  # obstacle
                tentative_g = gscore[current] + np.linalg.norm(np.array(current)-np.array(neighbor))
                if neighbor in close_set and tentative_g >= gscore.get(neighbor, 0):
                    continue
                if tentative_g < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g
                    fscore[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

def generate_collision_aware_path(start, goal, occupancy_grid, resolution, origin=(0,0)):
    """
    Convert world coordinates to grid indices, run A*, then convert back.

    Args:
        start (tuple): (x, y) in meters
        goal (tuple): (x, y) in meters
        occupancy_grid (np.ndarray): 2D occupancy grid
        resolution (float): meters per cell
        origin (tuple): (x, y) origin of the map

    Returns:
        path (list of tuples): list of (x, y) waypoints
    """
    # Convert world coordinates to grid indices
    start_idx = (int((start[1]-origin[1])/resolution), int((start[0]-origin[0])/resolution))
    goal_idx = (int((goal[1]-origin[1])/resolution), int((goal[0]-origin[0])/resolution))

    path_idx = astar(occupancy_grid, start_idx, goal_idx)
    if path_idx is None:
        return []  # no path found

    # Convert back to world coordinates
    path = []
    for i,j in path_idx:
        x = j * resolution + origin[0] + resolution/2
        y = i * resolution + origin[1] + resolution/2
        path.append((x, y))
    return path
