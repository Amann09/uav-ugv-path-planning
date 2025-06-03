import math

def points_in_square(waypoints, center_x, center_y, half_length):
    points_inside = []

    # Iterate through all waypoints
    for x, y in waypoints:
        if abs(x - center_x) <= half_length and abs(y - center_y) <= half_length:
            points_inside.append((x, y))

    return points_inside

def points_in_multiple_squares(waypoints, squares, half_length):
    results = []
    used_points = set()

    for i, (center_x, center_y) in enumerate(squares):
        points_inside = []

        for x, y in waypoints:
            if (x, y) not in used_points and abs(x - center_x) <= half_length and abs(y - center_y) <= half_length:
                points_inside.append((x, y))
                used_points.add((x, y))

        results.append((f"Square {i + 1}: Center {(center_x, center_y)}", points_inside))

    return results


if __name__ == "__main__":
    # Example usage
    waypoints = [(1, 2), (3, 4), (5, 6), (0, 0), (-1, -1)]
    squares = [
        (0, 0),  # Square 1: center (0, 0)
        (3, 3)   # Square 2: center (3, 3)
    ]
    half_length = 2  # Half of the side length

    # Get points inside each square
    results = points_in_multiple_squares(waypoints, squares, half_length)

    for square, points in results:
        print(f"{square}: {len(points)} points inside")
        print("Points:", points)
