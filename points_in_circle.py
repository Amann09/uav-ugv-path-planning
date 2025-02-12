import math

def points_in_circle(waypoints, center_x, center_y, radius):
    points_inside = []

    # Iterate through all waypoints
    for x, y in waypoints:
        # Check if the point lies inside the circle
        if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
            points_inside.append((x, y))

    return points_inside

def points_in_multiple_circles(waypoints, circles, radius):
    results = []
    used_points = set()

    for i, (center_x, center_y) in enumerate(circles):
        points_inside = []

        for x, y in waypoints:
            if (x, y) not in used_points and (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
                points_inside.append((x, y))
                used_points.add((x, y))

        results.append((f"Circle {i + 1}: Center {(center_x, center_y)}", points_inside))

    return results


if __name__ == "__main__":

    # Example usage
    waypoints = [(1, 2), (3, 4), (5, 6), (0, 0), (-1, -1)]
    circles = [
        (0, 0),  # Circle 1: center (0, 0), radius 5
        (3, 3)   # Circle 2: center (3, 3), radius 2
    ]

    # Get points inside each circle
    results = points_in_multiple_circles(waypoints, circles)

    for circle, points in results:
        print(f"{circle}: {len(points)} points inside")
        print("Points:", points)
