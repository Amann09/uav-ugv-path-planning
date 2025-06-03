import matplotlib.pyplot as plt
import networkx as nx

def setup_plot(Graph, figsize=(200, 200)):
    """Initialize plot and return node positions."""
    plt.figure(figsize=figsize)
    pos = {n: n for n in Graph.nodes()}
    return pos

def plot_possible_ugv_points(gvPoints):
    for points in gvPoints:
        x_coords, y_coords = zip(*points)
        plt.scatter(x_coords, y_coords, color='red', s=40)

def plot_grid_graph(Graph, pos, peripheral_nodes):
    # Draw main grid nodes
    nx.draw(Graph, pos, with_labels=True, node_size=30, node_color='skyblue', font_size=10, font_color='black', alpha=0.35)

    # Draw peripheral nodes
    nx.draw_networkx_nodes(Graph, pos, nodelist=peripheral_nodes, node_size=300, node_color='lightgreen')

def plot_hitting_set_circles(min_hitting_set, radius):
    for (x, y) in min_hitting_set:
        plt.scatter(x, y, color='red', marker='x', s=40)
        circle = plt.Circle((x, y), radius, color='red', linestyle='dotted', fill=False)
        plt.gca().add_patch(circle)

def finalize_plot(grid_size, tether_length):
    plt.scatter([], [], marker='>', label=f"grid size: {grid_size}x{grid_size}")
    plt.scatter([], [], marker='>', label=f"tether length: {tether_length} unit")
    plt.scatter([], [], color='red', s=40, label="Ground Vehicles Points")
    plt.scatter([], [], color='lightblue', s=100, label="UAV Points")

    plt.title(f"{grid_size}x{grid_size} Grid Graph with Peripheral Nodes")
    plt.legend()
    plt.axis('equal')
    plt.show()
