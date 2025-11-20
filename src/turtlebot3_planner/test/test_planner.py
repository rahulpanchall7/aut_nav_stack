import pytest
import numpy as np
from turtlebot3_planner.path_generator import generate_collision_aware_path
from turtlebot3_planner.path_smoother import smooth_path

def test_empty_grid_path():
    grid = np.zeros((10, 10), dtype=np.int8)
    start = (0.0, 0.0)
    goal = (0.9, 0.9)
    path = generate_collision_aware_path(start, goal, grid, resolution=0.1, origin=(0, 0))
    assert path, "Path should exist in empty grid"
    assert np.all(np.array(path) >= 0), "All path coordinates should be non-negative"

def test_obstacle_avoidance():
    grid = np.zeros((10, 10), dtype=np.int8)
    grid[5, 0:9] = 1  # horizontal wall with a gap at column 9
    start = (0.0, 0.0)
    goal = (0.9, 0.9)
    
    path = generate_collision_aware_path(start, goal, grid, resolution=0.1, origin=(0, 0))
    
    assert path, "Path should exist even with obstacle"
    
    # Ensure path does not go through obstacle
    for x, y in path:
        i = int(y / 0.1)
        j = int(x / 0.1)
        # Ensure indices are within bounds
        i = min(i, grid.shape[0] - 1)
        j = min(j, grid.shape[1] - 1)
        assert grid[i, j] == 0, "Path goes through obstacle!"


def test_smooth_path_length():
    waypoints = [(0,0), (0.5,0.5), (1.0,1.0)]
    smooth = smooth_path(waypoints, num_points=5)
    assert len(smooth) == 5, "Smoothed path should have requested number of points"
