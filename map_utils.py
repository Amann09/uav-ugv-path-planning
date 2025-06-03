import numpy as np
from geometry_utils import eucl_dist

def calculate_starting_point(reference_coordinate, points):
    candidates = [
        point + (round(eucl_dist(reference_coordinate, point), 3),)
        for point in points
    ]
    sorted_candidates = sorted(candidates, key=lambda x: x[2])
    starting_point = (sorted_candidates[0][0], sorted_candidates[0][1])
    return starting_point

def store_map(map, grid_size, idx, folder="Maps"):
    filename = f"{folder}/maps_{grid_size}x{grid_size}_{idx+1}.npy"
    np.save(filename, map)