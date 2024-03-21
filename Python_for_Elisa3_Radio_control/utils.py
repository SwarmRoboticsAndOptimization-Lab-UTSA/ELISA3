import tensorflow as tf
import numpy as np
import platform
from typing import List, NamedTuple
import subprocess

import json

import cv2
import math

Interpreter = tf.lite.Interpreter
load_delegate = tf.lite.experimental.load_delegate

_MARGIN = 5  # pixels
_ROW_SIZE = 5  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

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



def find_and_remove_closest_point(points, target_point):
    """
    Finds and removes the closest point to the target_point from a list of points.

    Parameters:
    - points: A list of (x, y) tuples.
    - target_point: A tuple containing the (x, y) coordinates of the target point.

    Returns:
    - A tuple containing two elements:
        1. The modified list with the closest point removed.
        2. The closest point as a tuple.
    """
    # Calculate Euclidean distance to each point and store it along with the index
    distances = [(math.sqrt((x - target_point[0]) ** 2 + (y - target_point[1]) ** 2), i)
                 for i, (x, y) in enumerate(points)]
    
    # Find the closest point (minimum distance) and its index
    _, closest_point_index = min(distances, key=lambda x: x[0])
    
    # Extract the closest point
    closest_point = points.pop(closest_point_index)
    
    # Return the modified list and the closest point
    return points, closest_point


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
    # Optionally, apply strength scaling
    attractive_force = (vector_to_goal[0] * strength, vector_to_goal[1] * strength)
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
            "v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_time_absolute=25",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=100",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=10",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=zoom_absolute=100",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=focus_automatic_continuous=0",
            "v4l2-ctl --device=/dev/video0 --set-ctrl=focus_absolute=0",
        ]
        for cmd in commands:
            subprocess.check_call(cmd, shell=True)
    except subprocess.CalledProcessError:
        print("Error executing camera setup commands.")



def get_formations_list():
  formations_list = [
    [ #U Formation
    (150, 150), # Top left
    (150, 250), # Mid left
    (150, 350), # Mid2 left
    (150, 450), # Bottom left
    (250, 450), # Bottom, starting to move right
    (350, 450), # Bottom, further right
    (450, 450), # Bottom, even further right
    (450, 350), # Bottom right
    (450, 250), # Mid right
    (450, 150), # Top right
    ],
    [ #T Formation
    (150,150), # Top left 1
    (210,150), # Top left 2
    (270,150), # Mid 1
    (330,150), # Mid 2
    (390,150), # Top right 1
    (450,150), # Top right 2
    (300,225), # Mid 3
    (300,300), # Mid 4
    (300,375), # Mid 5
    (300,450)  # Mid Bottom
    ],
    [ #S Formation
    (350, 60), # Top
    (205, 120), # 1 second row
    (495, 120),# 2 second row
    (205, 210), # 1 third row
    (350, 250), #  1 fourth row
    (360, 290), # 1 fifth row
    (360, 400), #  1 sixt row
    (337, 450), # 1 last row
    (263, 450), # 2 last row
    (225, 450) #  3 last row
    ],
    [ #A Formation
    (300,150),
    (225,213),
    (375,213),
    (150,321),
    (225,321),
    (300,321),
    (375,321),
    (450,321),
    (150,440),
    (450,440),
    ]
  ]
  return formations_list