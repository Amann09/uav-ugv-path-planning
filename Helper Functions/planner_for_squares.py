import networkx as nx
import matplotlib.pyplot as plt1
import numpy as np
import math
import ortools_hitting_set
import points_in_square
import re
from tsp import solve_tsp

def eucl_dist(P1, P2, z=None):
    (x1,y1) = P1
    (x2,y2) = P2
    if z is None:
        return math.sqrt( (x1-x2)**2 + (y1-y2)**2 )
    else:
        return math.sqrt( (x1-x2)**2 + (y1-y2)**2 + z**2)
        

def calc_eqOfLine_coeff(P1, P2):
    (x1, y1) = P1
    (x2, y2) = P2

    a = y2 - y1
    b = x1 - x2
    c = (x2*y1) - (x1*y2) 
    return a, b, c

def line_intersects_with_circle(coeff, center, radius):
    # This functions checks if a given line intersects with a given circle.
    a, b, c = coeff[0], coeff[1], coeff[2]
    x, y = center[0], center[1]

    d = ((abs(a * x + b * y + c)) / math.sqrt(a * a + b * b))
    return radius > d


def calculate_radius_of_base_circle(l, h):                 # l = slant height (tether length) and h = height (altitude)
    return round(math.sqrt(l**2 - h**2), 3)

def plot_possible_ugv_points(gvPoints):
    for points in gvPoints:
        x_coords, y_coords = zip(*points)
        plt1.scatter(x_coords, y_coords, color='red', s=40)
    
def first_to_fourth_quadrant(coord_list):
    new_coord_list = list()
    for (x, y) in coord_list:
        coord_x = y
        coord_y = grid_size - 1 - x
        new_coord_list.append((coord_x, coord_y))

    return new_coord_list

def fourth_to_first_quadrant(coord_list):
    new_coord_list = list()
    for (x, y) in coord_list:
        coord_x = grid_size - 1 - y
        coord_y = x
        new_coord_list.append((coord_x, coord_y))

    return new_coord_list

def calculate_starting_point(reference_coordinate, points):
    t = list()
    for point in points:
        dist = round(eucl_dist(reference_coordinate, point), 3)
        t.append(point + (dist, ))
    sorted_t = sorted(t, key=lambda x: x[2])[0]
    starting_point = (sorted_t[0], sorted_t[1])
    return starting_point

def store_map(map, i):
    np.save(f"Maps/maps_{grid_size}x{grid_size}_{i+1}", map)

def four_corners_of_square(center, r):
    point_a = (center[0] + (r/2), center[1] + (r/2))
    point_b = (center[0] - (r/2), center[1] + (r/2))
    point_c = (center[0] - (r/2), center[1] - (r/2))
    point_d = (center[0] + (r/2), center[1] - (r/2))
    return [point_a, point_b, point_c, point_d]

def calculate_radius_of_base_square(L, h):
    return ((L**2 - h**2)/2)

def do_lines_intersect(p1, p2, q1, q2):
    def ccw(a, b, c):
        return (c[1]-a[1]) * (b[0]-a[0]) > (b[1]-a[1]) * (c[0]-a[0])

    return (ccw(p1, q1, q2) != ccw(p2, q1, q2)) and (ccw(p1, p2, q1) != ccw(p1, p2, q2))

def line_intersects_with_sqaure(P, Q, center, radius):
    square_vertices = four_corners_of_square(center, radius)
    for i in range(4):
        square_p1 = square_vertices[i]
        square_p2 = square_vertices[(i + 1) % 4]  # wrap around
        if do_lines_intersect(P, Q, square_p1, square_p2):
            return True
    return False

def is_point_inside_square(K, center, r):
    x, y = K
    cx, cy = center
    
    if (cx - r <= x <= cx + r) and (cy - r <= y <= cy + r):
        return True
    else:
        return False

## MAIN CODE ## 
tether_length = 3                                                   # Slant Height (diagonal)
height = 2                                                          # Altitude
radius = calculate_radius_of_base_square(tether_length, height)     # Radius of the circular base 
grid_size = 10

Graph = nx.grid_2d_graph(grid_size, grid_size)
waypoints =  list(Graph.nodes())

# print(f"waypoints: {waypoints}")
# print(f"len(waypoints): {len(waypoints)}") # Output: 100

# origin, here, grid_size = n
(o, d) = (-1, grid_size)

# Define peripheral nodes 
"""  
d = n-1, o =  0 for nodes to be on the grid 
d = n,   o = -1 for nodes to be 1 unit away from the grid
d = n+1, o = -2 for nodes to be 2 unit away from the grid
d = n-2, o = +1 for nodes to be 1 unit inside the grid 
"""
peripheral_nodes = [(o, d)      , ((o+d)/2, d),     (d, d), 
                    (o, (o+d)/2),                   (d, (o+d)/2),
                    (o, o)      , ((o+d)/2, o),     (d, o)]

# Add peripheral nodes to the graph
for p_node in peripheral_nodes:
    Graph.add_node(p_node)

green_edges = [(peripheral_nodes[i], peripheral_nodes[j]) for i in range(len(peripheral_nodes)) for j in range(i + 1, len(peripheral_nodes))]

green_edges_with_angles = []
for edges in green_edges:
    # print(f"current {edges}, {len(green_edges)}")
    x1, y1 = edges[0][0], edges[0][1]
    x2, y2 = edges[1][0], edges[1][1]

    angle = math.atan2((y2-y1),(x2-x1))

    green_edges_with_angles.append(edges + (angle, ))

new_green_edges = []
for edges in green_edges_with_angles:
    if abs(edges[2]) not in [float(0), math.pi/2, math.pi]:
        new_green_edges.append(edges)

# Draw the grid graph
plt1.figure(figsize=(200, 200))
pos = dict((n, n) for n in Graph.nodes())  # Positions as grid coordinates

# Draw grid nodes
nx.draw(Graph, pos, with_labels=True, node_size=30, node_color='skyblue', font_size=10, font_color='black', alpha=0.35)

# Draw peripheral nodes and green edges
nx.draw_networkx_nodes(Graph, pos, nodelist=peripheral_nodes, node_size=300, node_color='lightgreen')


Lines = [[(o, d),       (d, (o+d)/2)],
         [(o, d),       (d, o)],
         [(o, d),       ((o+d)/2, o)],
         [(o, (o+d)/2), ((o+d)/2, d)],
         [(o, (o+d)/2), (d, d)],
         [(o, (o+d)/2), (d, o)],
         [(o, (o+d)/2), ((o+d)/2, o)],
         [(o, o),       ((o+d)/2, d)],
         [(o, o),       (d, d)],
         [(o, o),       (d, (o+d)/2)],
         [((o+d)/2, d), (d, (o+d)/2)],
         [((o+d)/2, o), (d, (o+d)/2)],
         [((o+d)/2, o), (d, d)],
         [((o+d)/2, d), (d, o)]
        ]

gvPoints = []
for line in Lines:
    (x1, y1) = line[0]
    (x2, y2) = line[1]
    Dist = round(eucl_dist((x1, y1), (x2, y2)), 3)
    
    d = 1
    d_vector = ((x2-x1)/Dist, (y2-y1)/Dist)
    points = []
    for n in range(1, round(Dist/d)):
        x_n = x1 + n * d * d_vector[0]
        y_n = y1 + n * d * d_vector[1]
        x_n, y_n = round(x_n, 3), round(y_n, 3)
        points.append((x_n, y_n))

    # gvPoints.append(((x1, y1), (x2, y2), points))
    gvPoints.append(points)

# print(f"gvPoints: {gvPoints}")
# print("\n")

# # Plotting the possible UGV points # 
# plot_possible_ugv_points(gvPoints)

s_of_s = []
for waypoint in waypoints:
    (x, y) = waypoint
    intersecting_lines = []
    for line in Lines:
        if line_intersects_with_sqaure(line[0], line[1], center=(x, y), radius=radius):
            intersecting_lines.append(line)
    
    # print(f"For waypoint: {waypoint} Intersecting lines: \n {intersecting_lines}")

    # Below code checks how many points in gvPoints that lies in the circle when intersecting with a line.
    s = []
    for line in intersecting_lines:
        idx = Lines.index(line)
        points = gvPoints[idx]
        for point in points:
            if is_point_inside_square(point, center=(x, y), r=radius):
                s.append(point)
    
    s_of_s.append(s)

    # print(s, len(s))
    # print("\n")
    # print(s_of_s, len(s_of_s))

    # if waypoints.index(waypoint) > 3:
    #     break

# print(f"s_of_s: {s_of_s}")


min_hitting_set = ortools_hitting_set.hitting_set_with_ortools(s_of_s)

for (x, y) in min_hitting_set:
    plt1.scatter(x, y, color='blue', marker='s', s=40)

    half_length = radius  # assuming your computed radius is half side length
    square = plt1.Rectangle(
        (x - half_length, y - half_length),
        2 * half_length, 2 * half_length,
        edgecolor='blue',
        linestyle='dotted',
        linewidth=1.0,
        fill=False
    )
    plt1.gca().add_patch(square)


circles = list(min_hitting_set)
results = points_in_square.points_in_multiple_squares(waypoints, circles, half_length=radius)

print(f"results: {results}")
print(type(results), len(results))

circle_data = dict()

for circle_number, circle_points in results:
    # print(f"circle_number: {circle_number} \ncircle_points: {circle_points}")
    center_coords = tuple(map(float, re.search(r'\(([^,]+),\s*([^)]+)\)', circle_number).groups()))
    circle_data[center_coords] = circle_points

print(f"circle_data: {circle_data}")
circle_data = {k: v for k, v in circle_data.items() if v}
print(f"circle_data after cleanup: {circle_data}")

circle_centers  = solve_tsp((0, 0), list(circle_data.keys()))
circle_waypoints = [circle_data[value] for value in circle_centers]


# circle_waypoints.reverse()
# circle_centers.reverse()

print("")
print(f"circle_centers: {circle_centers}")
print(f"circle_waypoints: {circle_waypoints}")

np_map = np.ones((grid_size, grid_size), dtype=int)

reference_coordinate = (0, 0)
num = 0
starting_point_list = []
obstacles = [(3, 2), (7, 6)]
for waypoints in circle_waypoints:
    starting_point = calculate_starting_point(reference_coordinate, waypoints)
    for (x, y) in waypoints:
        if (x, y) == starting_point:
            np_map[x][y] = 2
        elif (x, y) in obstacles:
            np_map[x][y] = 3
        else:
            np_map[x][y] = 0
    starting_point_list.append(starting_point)
    reference_coordinate = starting_point
        
    print(np_map)
    store_map(np_map, num)
    # np_map[starting_point[0]][starting_point[1]] = 0
    num += 1
    
    np_map = np.ones((grid_size, grid_size), dtype=int)

print(starting_point_list)

plt1.scatter([], [], marker='>', label=f"grid size: {grid_size}x{grid_size}")
plt1.scatter([], [], marker='>', label=f"tether length: {tether_length} unit")
plt1.scatter([], [], color='red', s=40, label="Ground Vehicles Points")
plt1.scatter([], [], color='lightblue', s=100, label="UAV Points")

# Show the plot
plt1.title(f"{grid_size}x{grid_size} Grid Graph with Peripheral Nodes")
plt1.legend()
plt1.axis('equal')
plt1.show()
