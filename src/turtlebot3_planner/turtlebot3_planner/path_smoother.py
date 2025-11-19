#Functions for cubic spline smoothing of a discrete path
import numpy as np
from scipy.interpolate import CubicSpline

def smooth_path(path, num_points=200):
    """
    Smooth a discrete path using cubic spline interpolation.

    Args:
        path (list of tuples): discrete waypoints [(x, y), ...]
        num_points (int): number of points in the smoothed path

    Returns:
        smoothed_path (list of tuples): smooth waypoints [(x, y), ...]
    """
    if len(path) < 3:
        # Too few points to smooth; return original
        return path

    path = np.array(path)

    # Compute cumulative distance along the path
    distances = np.zeros(path.shape[0])
    distances[1:] = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))

    # Cubic spline interpolation
    cs_x = CubicSpline(distances, path[:,0])
    cs_y = CubicSpline(distances, path[:,1])

    # Sample the spline at regular intervals
    t_new = np.linspace(0, distances[-1], num_points)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)

    smoothed_path = list(zip(x_new, y_new))
    return smoothed_path


def compute_orientations(smoothed_path):
    """
    Compute orientation (theta) at each waypoint of a 2D path.

    Args:
        smoothed_path (list of tuples): [(x, y), ...]

    Returns:
        list of tuples: [(x, y, theta), ...] where theta is in radians
    """
    smoothed_path = np.array(smoothed_path)
    dx = np.gradient(smoothed_path[:,0])
    dy = np.gradient(smoothed_path[:,1])
    theta = np.arctan2(dy, dx)
    return list(zip(smoothed_path[:,0], smoothed_path[:,1], theta))

