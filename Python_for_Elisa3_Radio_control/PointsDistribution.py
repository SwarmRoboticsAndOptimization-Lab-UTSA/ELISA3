import cv2
import numpy as np

def calculate_distance(p1, p2):
    """Calculate the Euclidean distance between two points."""
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

def distribute_points(paths, num_points):
    """
    Distribute points evenly along a series of lines defined by start and end points.
    :param paths: List of tuples, where each tuple contains start and end points (x, y) of a line.
    :param num_points: Total number of points to distribute.
    :return: List of point coordinates (x, y) where points should be placed.
    """
    # Calculate the total length of the paths
    total_length = sum(calculate_distance(start, end) for start, end in paths)
    
    # Calculate distance between points
    distance_between_points = total_length / (num_points - 1)
    
    points = []
    accumulated_distance = 0
    
    for start, end in paths:
        if not points:
            # Add the first point
            points.append(start)
            last_point = start
        else:
            last_point = points[-1]

        # Calculate the length of the current line
        line_length = calculate_distance(start, end)
        while accumulated_distance + distance_between_points <= line_length:
            # Find the next point along the line
            ratio = (accumulated_distance + distance_between_points) / line_length
            next_point = (start[0] + ratio * (end[0] - start[0]), start[1] + ratio * (end[1] - start[1]))
            points.append(next_point)
            # Update the accumulated distance
            accumulated_distance += distance_between_points
            last_point = next_point

        # Update the accumulated distance for the next line
        accumulated_distance -= line_length

    # Ensure the last point is always added
    if len(points) < num_points:
        points.append(paths[-1][1])

    return points

# Define the paths for the letter U
paths = [((80, 40), (80, 430)), ((80, 430), (570, 430)), ((570, 430), (570, 40))]

# Distribute 10 points
points = distribute_points(paths, 15)

print(type(points))
print(points)

# Create an image to draw
image = np.zeros((500, 650, 3), dtype=np.uint8)
for point in points:
    cv2.circle(image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)

cv2.imshow("Letter U with Points", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
