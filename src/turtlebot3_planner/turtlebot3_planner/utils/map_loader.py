import yaml
import os
from PIL import Image
import numpy as np

def load_map_from_yaml(yaml_file):
    """
    Load a 2D occupancy grid from a ROS2 map.yaml + map.pgm.

    Args:
        yaml_file (str): Path to the map.yaml file.

    Returns:
        occupancy_grid (np.ndarray): 2D array, 0=free, 1=occupied
        resolution (float): meters per cell
        origin (tuple): (x, y) origin of the map in world coordinates
    """
    with open(yaml_file, 'r') as f:
        map_data = yaml.safe_load(f)

    resolution = map_data['resolution']
    origin = tuple(map_data['origin'][:2])  # x, y

    # Get full path to PGM file
    pgm_file = os.path.join(os.path.dirname(yaml_file), map_data['image'])

    # Load PGM image and convert to numpy array
    img = Image.open(pgm_file)
    img = np.array(img)

    # Convert to occupancy grid: 0=free, 1=occupied
    occupancy_grid = np.zeros_like(img, dtype=np.int8)
    occupancy_grid[img < 127] = 1  # treat dark pixels as obstacles

    return occupancy_grid, resolution, origin
