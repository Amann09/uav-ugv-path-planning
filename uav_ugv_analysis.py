import numpy as np
import re
from tsp import nearest_neighbor_tsp
from map_utils import calculate_starting_point, store_map


def parse_circle_data(results):
    circle_data = {
        tuple(map(float, re.search(r'\(([^,]+),\s*([^)]+)\)', circle_str).groups())): points
        for circle_str, points in results
    }
    return circle_data


def analyze_uav_ugv_coverage(results, grid_size, tether_length, reference_coordinate=(0, 0)):
    """
    Processes UAV/UGV coverage data and prepares paths and maps for execution.

    Args:
        results (list): Output from `points_in_multiple_circles()`
        grid_size (int): Size of the grid map
        tether_length (float): Length of the UAV tether
        reference_coordinate (tuple): Starting reference for UGV

    Returns:
        tuple:
            - circle_centers: Ordered list of circle center coordinates
            - circle_waypoints: List of waypoints for each circle
            - starting_point_list: Chosen UGV starting points for each circle
    """
    circle_data = parse_circle_data(results)

    circle_centers = nearest_neighbor_tsp(reference_coordinate, list(circle_data.keys()))
    circle_waypoints = [circle_data[center] for center in circle_centers]

    np_map = np.ones((grid_size, grid_size), dtype=int)
    num = 0
    starting_point_list = []

    for waypoints in circle_waypoints:
        starting_point = calculate_starting_point(reference_coordinate, waypoints)

        for (x, y) in waypoints:
            if (x, y) == starting_point:
                np_map[x][y] = 2
            else:
                np_map[x][y] = 0

        starting_point_list.append(starting_point)
        reference_coordinate = starting_point

        print(np_map)
        store_map(np_map, grid_size, num)
        num += 1
        np_map = np.ones((grid_size, grid_size), dtype=int)

    return circle_centers, circle_waypoints, starting_point_list
