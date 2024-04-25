import tensorflow as tf
import numpy as np
import platform
from typing import List, NamedTuple
import subprocess

import json

import cv2
import math

def midpoint(point1, point2): #Function to calculate the mid point between two points
    x1, y1 = point1
    x2, y2 = point2

    xm = (x1 + x2) / 2
    ym = (y1 + y2) / 2

    return (int(xm), int(ym))

# # Example usage:
# point1 = (1, 2)
# point2 = (3, 4)
# print(midpoint(point1, point2))  # Output: (2.0, 3.0)

def calculate_heading(point1, point2): #Function to calculate the heading based on two points
    x1, y1 = point1
    x2, y2 = point2

    theta = math.atan2(y2 - y1, x2 - x1)
    bearing = math.degrees(theta)
    
    # Convert from [-180, 180] to [0, 360]
    if bearing < 0:
        bearing += 360

    return bearing

# Example usage:
# point1 = (0, 0)
# point2 = (1, 1)
# print(calculate_heading(point1, point2))  # Should be roughly 45.0


def calculate_distance(x1, y1, x2, y2):
    """
    Calculate the squared differences in x and y coordinates
    """
    x_diff_squared = (x2 - x1) ** 2
    y_diff_squared = (y2 - y1) ** 2
    
    # Calculate the sum of squared differences and then take the square root
    distance = math.sqrt(x_diff_squared + y_diff_squared)
    
    return distance

# Example usage:
# x1, y1 = 0, 0  # Coordinates of the first point
# x2, y2 = 3, 4  # Coordinates of the second point
# distance = calculate_distance(x1, y1, x2, y2)
# print(f"The distance between the two points is {distance:.2f} units.")
def calculate_distance_point(p1, p2):
    """Calculate the Euclidean distance between two points."""
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5



def calculate_attractive_force(current_position, goal_position, strength=1.0):
    """
    Calculate the attractive force vector towards the goal.
    :param current_position: Tuple (x, y) of the robot's current position.
    :param goal_position: Tuple (x, y) of the goal position.
    :param strength: Scalar that determines the strength of the attraction.
    :return: Tuple representing the attractive force vector (x, y).
    """
    # Vector from current to goal
    vector_to_goal = (goal_position[0] - current_position[0], goal_position[1] - current_position[1])
    # Calculate the magnitude (length) of the vector
    magnitude = (vector_to_goal[0]**2 + vector_to_goal[1]**2)**0.5
    # Normalize the vector to have a length of 1, then scale by strength
    if magnitude != 0:
        normalized_vector_to_goal = (vector_to_goal[0] / magnitude, vector_to_goal[1] / magnitude)
    else:
        normalized_vector_to_goal = (0, 0)  # Handle the case where current_position == goal_position
    attractive_force = (normalized_vector_to_goal[0] * strength, normalized_vector_to_goal[1] * strength)
    return attractive_force

def calculate_repulsive_force(current_position, obstacle_position, sensor_range, strength=1.0):
    """
    Calculate the repulsive force vector away from an obstacle.
    :param current_position: Tuple (x, y) of the robot's current position.
    :param obstacle_position: Tuple (x, y) of the obstacle's position.
    :param sensor_range: Maximum range of the proximity sensor.
    :param strength: Scalar that determines the strength of the repulsion.
    :return: Tuple representing the repulsive force vector (x, y).
    """
    distance = calculate_distance(current_position[0], current_position[1], obstacle_position[0], obstacle_position[1])
    if distance < sensor_range:
        # Calculate vector from obstacle to robot
        vector_away_from_obstacle = (current_position[0] - obstacle_position[0], current_position[1] - obstacle_position[1])
        # Scale based on distance to obstacle and strength
        scale = strength * (1 - (distance / sensor_range)) ** 2  # Quadratic scaling for stronger repulsion at close range

        repulsive_force = (vector_away_from_obstacle[0] * scale, vector_away_from_obstacle[1] * scale)
        return repulsive_force
    else:
        return (0, 0)


# Camera and Detector Setup
def initialize_camera():
    try:
        subprocess.check_call(["v4l2-ctl", "--set-ctrl=auto_exposure=1"])
        commands = [
            "v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_time_absolute=155",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=76",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=0",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=zoom_absolute=100",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=focus_automatic_continuous=0",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=focus_absolute=0",
        ]
        for cmd in commands:
            subprocess.check_call(cmd, shell=True)
    except subprocess.CalledProcessError:
        print("Error executing camera setup commands.")

def extract_tag_info(tag, debug_image):
    tag_identifier = (tag.tag_family, tag.tag_id)  # Unique identifier for each tag
    #tag_family = tag.tag_family
    #tag_id = tag.tag_id
    center = tag.center
    corners = tag.corners
    center = (int(center[0]), int(center[1]))
    corner_01 = (int(corners[0][0]), int(corners[0][1]))
    corner_02 = (int(corners[1][0]), int(corners[1][1]))
    # corner_03 = (int(corners[2][0]), int(corners[2][1]))
    # corner_04 = (int(corners[3][0]), int(corners[3][1]))
    mid = midpoint(corner_01,corner_02)
    cv2.line(debug_image, (center[0], center[1]),(mid[0], mid[1]), (255, 255, 0), 2)

    return tag_identifier, center, corners, mid,debug_image


def assign_unique_goals(tags,current_formation):
    distances = [] 

    for tag in tags:
        center = tag.center
        corners = tag.corners
        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        mid = midpoint(corner_01,corner_02)
        # print(tag.tag_id, mid)
        for goal in current_formation:
            distance_to_goal = calculate_distance(mid[0], mid[1], goal[0], goal[1])
            distances.append((distance_to_goal, tag.tag_id, goal))
    
    distances.sort()
    assigned_goals = set()
    robot_goal_pairs = {}

   # Assign goals to robots, ensuring no goal is assigned twice
    for distance, robot_id, goal in distances:
        if goal not in assigned_goals and robot_id not in robot_goal_pairs:
            assigned_goals.add(goal)
            robot_goal_pairs[robot_id] = goal

    return robot_goal_pairs


def create_lattice(n, width, height, margin):
    # Determine the number of rows and columns for the lattice
    cols = int(math.sqrt(n))
    rows = cols if cols * cols == n else cols + 1
    
    # Adjust columns if necessary to fit all points
    while rows * cols < n:
        cols += 1
    
    # Calculate the spacing between points to fit within the frame minus margins
    x_spacing = (width - 2 * margin) / (cols - 1) if cols > 1 else 0
    y_spacing = (height - 2 * margin) / (rows - 1) if rows > 1 else 0

    # Generate the points centered within the margins
    points = []
    for i in range(rows):
        for j in range(cols):
            if len(points) < n:
                x = margin + j * x_spacing
                y = margin + i * y_spacing
                points.append((int(x), int(y)))
    
    return points

# Example usage:
# n = 10  # Number of points
# width = 640  # Width of the frame in pixels
# height = 480  # Height of the frame in pixels
# margin = 50  # Margin size in pixels
# points = create_lattice(n, width, height, margin)
# print(points)
