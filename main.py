import networkx as nx
import ortools_hitting_set
import points_in_circle

from config import tether_length, height, grid_size, o, d
from geometry_utils import *
from map_utils import *
from plotting import (
    setup_plot,
    plot_possible_ugv_points,
    plot_grid_graph,
    plot_hitting_set_circles,
    finalize_plot
)
from uav_ugv_analysis import analyze_uav_ugv_coverage


def main():
    radius = calculate_radius(tether_length, height)    # Radius of the circular base 

    Graph = nx.grid_2d_graph(grid_size, grid_size)
    waypoints =  list(Graph.nodes())

    peripheral_nodes = generate_peripheral_nodes(o, d)

    # Add peripheral nodes to the graph
    for p_node in peripheral_nodes:
        Graph.add_node(p_node)

    # green_edges = generate_non_cardinal_edges(peripheral_nodes)

    # Draw the grid graph
    pos = setup_plot(Graph)
    plot_grid_graph(Graph, pos, peripheral_nodes)


    Lines = generate_lines(o, d)


    gvPoints = compute_gv_points(Lines)
    print(f"gvPoints: {gvPoints}")
    print("\n")


    # # Plotting the possible UGV points # 
    # plot_possible_ugv_points(gvPoints)

    ugv_candidate_sets = generate_ugv_candidate_sets(waypoints, Lines, gvPoints, radius)

    min_hitting_set = ortools_hitting_set.hitting_set_with_ortools(ugv_candidate_sets)

    plot_hitting_set_circles(min_hitting_set, radius)

    circles = list(min_hitting_set)
    results = points_in_circle.points_in_multiple_circles(waypoints, circles, radius=radius)

    # print(f"results: {results}")
    # print(type(results), len(results))

    circle_centers, circle_waypoints, starting_point_list = analyze_uav_ugv_coverage(
        results, grid_size, tether_length
    )

    print(f"\ncircle_centers: {circle_centers}")
    print(f"circle_waypoints: {circle_waypoints}")


    # Finalize and display the plot
    finalize_plot(grid_size, tether_length)

if __name__ == "__main__":
    main()