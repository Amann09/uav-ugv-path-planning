import itertools
import math
import networkx as nx
import numpy as np

def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def total_path_distance(path):
    total = 0
    for i in range(len(path) - 1):
        total += distance(path[i], path[i + 1])
    return total

def nearest_neighbor_tsp(start, waypoints):
    unvisited = waypoints.copy()
    path = [start]
    current = start

    while unvisited:
        next_point = min(unvisited, key=lambda p: distance(current, p))
        path.append(next_point)
        unvisited.remove(next_point)
        current = next_point

    return path[1:]

def two_opt(path):
    improved = True
    while improved:
        improved = False
        for i in range(1, len(path) - 2):
            for j in range(i + 1, len(path)):
                if j - i == 1:
                    continue
                new_path = path[:i] + path[i:j][::-1] + path[j:]
                if total_path_distance(new_path) < total_path_distance(path):
                    path = new_path
                    improved = True
        
    return path

def networkx_tsp_solver(points):
    G = nx.complete_graph(len(points))
    pos = {i: points[i] for i in G.nodes}
    for i in G.nodes:
        for j in G.nodes:
            if i != j:
                G[i][j]['weight'] = distance(points[i], points[j])

    tsp_cycle = nx.approximation.traveling_salesman_problem(G, weight='weight')
    ordered_points = [points[i] for i in tsp_cycle]
    return ordered_points

if __name__ == "__main__":
    start = (0, 0)
    waypoints = [(-0.106, 5.053), (4.814, 10.628), (6.071, 4.929), (3.243, 3.243),
                 (10.628, 4.814), (9.606, 1.394), (1.121, 9.879), (4.814, 0.372),
                 (8.192, 2.808), (9.606, 9.606), (8.192, 8.192), (3.025, 7.05), (1.121, 1.121)]

    print("--- Nearest Neighbor + 2-opt ---")
    initial_path = nearest_neighbor_tsp(start, waypoints)
    
    print(initial_path)
    # improved_path = two_opt(initial_path)
    # print(improved_path)
    # print(f"Total distance: {total_path_distance(improved_path):.3f}")

    # print("\n--- NetworkX Approximate TSP ---")
    # all_points = [start] + waypoints
    # nx_path = networkx_tsp_solver(all_points)
    # print(nx_path)
    # print(f"Total distance: {total_path_distance(nx_path):.3f}")
