import numpy as np
import cv2
import copy

def draw_letter_A(img, points):
    height, width = img.shape[:2]
    
    # Define the three main lines of A: left slant, right slant, and the crossbar
    start_left = (int(width * 0.2), height - 1)
    end_top = (int(width * 0.5), int(height * 0.2))
    start_right = (int(width * 0.8), height - 1)
    crossbar_start = (int(width * 0.3), int(height * 0.5))
    crossbar_end = (int(width * 0.7), int(height * 0.5))
    
    # Calculate points for each part
    points_left_slant = np.linspace(start_left, end_top, points//3, dtype=int)
    points_right_slant = np.linspace(start_right, end_top, points//3, dtype=int)
    points_crossbar = np.linspace(crossbar_start, crossbar_end, points//3, dtype=int)
    
    # Draw each point
    for point in points_left_slant:
        cv2.circle(img, tuple(point), 1, (255, 255, 255), -1)
    for point in points_right_slant:
        cv2.circle(img, tuple(point), 1, (255, 255, 255), -1)
    for point in points_crossbar:
        cv2.circle(img, tuple(point), 1, (255, 255, 255), -1)
        
    return img

def draw_letter_U(img, points, safe_zone=60):
    height, width = img.shape[:2]
    
    # Adjust the start and end points for the vertical lines, respecting the safe zone
    start_left = (safe_zone, safe_zone)
    end_left = (safe_zone, height - safe_zone)
    start_right = (width - safe_zone, safe_zone)
    end_right = (width - safe_zone, height - safe_zone)
    
    # Define the starting and ending points for the bottom horizontal line, within the safe zone
    bottom_start = (safe_zone, height - safe_zone)
    bottom_end = (width - safe_zone, height - safe_zone)
    
    # Distribute points among the vertical and bottom horizontal lines
    points_left_line = np.linspace(start_left, end_left, points // 3, dtype=int)
    points_right_line = np.linspace(start_right, end_right, points // 3, dtype=int)
    points_bottom_line = np.linspace(bottom_start, bottom_end, points // 3, dtype=int)
    
    # Draw each point
    for point in points_left_line:
        cv2.circle(img, tuple(point), 1, (255, 255, 255), -1)
    for point in points_right_line:
        cv2.circle(img, tuple(point), 1, (255, 255, 255), -1)
    for point in points_bottom_line:
        cv2.circle(img, tuple(point), 1, (255, 255, 255), -1)
        
    return img

def draw_letter_T_two_columns_with_spacing(img, points, min_distance, safe_zone=60):
    height, width = img.shape[:2]
    
    # Calculate available space after accounting for safe zones
    available_width = width - 2 * safe_zone
    available_height = height - 2 * safe_zone
    
    # Determine the number of points that can be placed on the horizontal and vertical lines given the minimum distance
    num_points_horizontal = min(points // 3, available_width // min_distance)
    num_points_vertical = min((points - num_points_horizontal) // 2, available_height // min_distance)

    # Adjust the start and end points for the top horizontal line and the two vertical center lines, within the safe zone
    top_start = (safe_zone, safe_zone)
    top_end = (safe_zone + num_points_horizontal * min_distance, safe_zone)
    
    center_vertical_start_y = safe_zone
    center_vertical_end_y = safe_zone + num_points_vertical * min_distance
    
    # Determine the spacing for the two columns
    center_column_x_left = width // 2 - min_distance // 2
    center_column_x_right = width // 2 + min_distance // 2
    
    # Generate points for the top horizontal line
    points_top_line = [(safe_zone + i * min_distance, safe_zone) for i in range(num_points_horizontal)]
    
    # Generate points for the left and right vertical lines
    points_center_line_left = [(center_column_x_left, center_vertical_start_y + i * min_distance) for i in range(num_points_vertical)]
    points_center_line_right = [(center_column_x_right, center_vertical_start_y + i * min_distance) for i in range(num_points_vertical)]
    
    # Draw each point
    for point in points_top_line:
        cv2.circle(img, point, 1, (255, 255, 255), -1)
    for point in points_center_line_left:
        cv2.circle(img, point, 1, (255, 255, 255), -1)
    for point in points_center_line_right:
        cv2.circle(img, point, 1, (255, 255, 255), -1)
        
    return img


# Assuming you're using the video capture logic provided earlier
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    debug_image = copy.deepcopy(frame)

    # Convert to grayscale for visibility of white "U"
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Number of points to use
    p = 10

    # Draw the letter
    img_with_letter = draw_letter_T_two_columns_with_spacing(image,p, 60)

    # Show the image
    cv2.imshow('Letter U', img_with_letter)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()