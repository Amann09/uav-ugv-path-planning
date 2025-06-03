import math
from config import o, d

def eucl_dist(P1, P2, z=None):
    x1, y1 = P1
    x2, y2 = P2
    if z is None:
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    else:
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + z**2)
    
def calc_eqOfLine_coeff(P1, P2):
    x1, y1 = P1
    x2, y2 = P2

    a = y2 - y1
    b = x1 - x2
    c = (x2*y1) - (x1*y2) 
    return a, b, c

def line_intersects_circle(coeff, center, radius):
    # This functions checks if a given line intersects with a given circle.
    a, b, c = coeff
    x, y = center

    distance = ((abs(a * x + b * y + c)) / math.sqrt(a * a + b * b))
    return radius > distance

def calculate_radius(l, h):                 # l = slant height (tether length) and h = height (altitude)
    return round(math.sqrt(l**2 - h**2), 3)

def generate_non_cardinal_edges(peripheral_nodes):
    green_edges = [
        (peripheral_nodes[i], peripheral_nodes[j])
        for i in range(len(peripheral_nodes))
        for j in range(i + 1, len(peripheral_nodes))
    ]

    green_edges_with_angles = []
    for edge in green_edges:
        x1, y1 = edge[0]
        x2, y2 = edge[1]
        angle = math.atan2((y2 - y1), (x2 - x1))
        green_edges_with_angles.append(edge + (angle,))

    non_cardinal_edges = [
        e for e in green_edges_with_angles
        if abs(e[2]) not in [0.0, math.pi / 2, math.pi]
    ]

    return non_cardinal_edges


def generate_lines(o, d):
    """Generate a fixed set of line segments used for UGV point generation."""
    return [[(o, d),       (d, (o + d) / 2)],
            [(o, d),       (d, o)],
            [(o, d),       ((o + d) / 2, o)],
            [(o, (o + d) / 2), ((o + d) / 2, d)],
            [(o, (o + d) / 2), (d, d)],
            [(o, (o + d) / 2), (d, o)],
            [(o, (o + d) / 2), ((o + d) / 2, o)],
            [(o, o),       ((o + d) / 2, d)],
            [(o, o),       (d, d)],
            [(o, o),       (d, (o + d) / 2)],
            [((o + d) / 2, d), (d, (o + d) / 2)],
            [((o + d) / 2, o), (d, (o + d) / 2)],
            [((o + d) / 2, o), (d, d)],
            [((o + d) / 2, d), (d, o)]]

def generate_peripheral_nodes(o, d):
    """  
        d = n-1, o =  0 for nodes to be on the grid 
        d = n,   o = -1 for nodes to be 1 unit away from the grid
        d = n+1, o = -2 for nodes to be 2 unit away from the grid
        d = n-2, o = +1 for nodes to be 1 unit inside the grid 
    """
    return [(o, d)      , ((o+d)/2, d),     (d, d), 
            (o, (o+d)/2),                   (d, (o+d)/2),
            (o, o)      , ((o+d)/2, o),     (d, o)]


# peripheral_nodes = generate_peripheral_nodes(o, d)
# Lines = generate_lines(o, d)

def compute_gv_points(Lines):
    gvPoints = []
    for line in Lines:
        (x1, y1), (x2, y2) = line
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
    return gvPoints

def generate_ugv_candidate_sets(waypoints, Lines, gvPoints, radius):
    candidate_sets = []
    for waypoint in waypoints:
        x, y = waypoint
        intersecting_lines = [
            line for line in Lines
            if line_intersects_circle(
                calc_eqOfLine_coeff(line[0], line[1]),
                (x, y), radius
            )
        ]
        s = []
        for line in intersecting_lines:
            idx = Lines.index(line)
            for point in gvPoints[idx]:
                if eucl_dist((x, y), point) < radius:
                    s.append(point)
        if s:
            candidate_sets.append(s)
    return candidate_sets