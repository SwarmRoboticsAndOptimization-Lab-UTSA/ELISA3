import elisa3
import time
import cv2
from pupil_apriltags import Detector
import copy
import subprocess
from NewObstacleAvoidanceUtils import *

initialize_camera()

at_detector = Detector(
        families="tagCustom48h12",
        nthreads=1,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

cap = cv2.VideoCapture(0)
processed_tags = set()  # Set to keep track of processed tags

robot_dict = {}
detected_robot_ids = set()
robotAddr = [4469,3829,3868,3904,4021,4050,3918,4101,4096,3828,3887,3823,3846,3890,3901]

# robotAddr = [3868,4050,3828,4060,4083,4021,3904,3829,3918,4469,4101,3988,4096,3948,4104]
# robotAddr = [3918,3868,3828]
formation_id = 0
formations = get_formations_list(len(robotAddr))

current_formation = formations[formation_id]
display_locations = current_formation.copy()
robot_status_dict = {robot_id: False for robot_id in robotAddr} #Status of location of the robots
elisa = elisa3.Elisa3(robotAddr)
elisa.start()

sensor_range = 80 #Sensor range for avoidance of obstacles
max_speed = 5 #The maximum speed of the robot
min_speed = -5
dynamic_sensor_range = sensor_range
min_sensor_range = 50  # Minimum sensor range when close to the goal
max_sensor_range = sensor_range  # Use the initially defined sensor range as the maximum
kp = 0.1 #Proportional gain for the rotation control
dead_zone = 5  # Threshold for the heading controller
close_threshold = 20  #Distance within robot start slowing down and reducing the control gain
stop_threshold = 8 #Distance within the robot stops moving
assigned_goals_bool = False

while True:
    ret, frame = cap.read()
    if not ret:
        break
    detected_robot_ids = set()
    debug_image = copy.deepcopy(frame)

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        image,
        estimate_tag_pose=False,
        camera_params=None,
        tag_size=None,
    )

    if assigned_goals_bool is True:
        pass
    else:
        # Assign unique goals to robots
        assigned_goals = assign_unique_goals(tags, current_formation)
        assigned_goals_bool = True
    
    for tag in tags:
        # print(tag.tag_id)
        tag_identifier, center, corners, mid, debug_image = extract_tag_info(tag,debug_image)
        heading = calculate_heading(center,mid)
        detected_robot_ids.add(tag.tag_id)  # Add the detected robot's ID to the set

        for robot_id, goal in assigned_goals.items():
            if tag.tag_id == robot_id:
                robot_dict[str(tag.tag_id)] = goal #Add robot and its desired location
        
        # Initialize repulsive force for current robot
        repulsive_force = (0, 0)

        # Calculate repulsive force from all other robots
        for other_tag in tags:
            if tag.tag_id != other_tag.tag_id:
                other_center = (int(other_tag.center[0]), int(other_tag.center[1]))
                # Calculate repulsive force from the current robot to the other robot
                obstacle_repulsion = calculate_repulsive_force(center, other_center, sensor_range=dynamic_sensor_range, strength=7)
                # Accumulate the repulsive forces
                repulsive_force = (repulsive_force[0] + obstacle_repulsion[0], repulsive_force[1] + obstacle_repulsion[1])

        # Calculate attractive force towards the goal
        if str(tag.tag_id) in robot_dict:
            goal_position = robot_dict[str(tag.tag_id)]
            # Calculate distance to goal
            distance_to_goal = calculate_distance(mid[0], mid[1], goal_position[0], goal_position[1])
            # Calculate attraction force
            attractive_force = calculate_attractive_force(center, goal_position, strength=1)
            # Calculate net force
            net_force = (attractive_force[0] + repulsive_force[0], attractive_force[1] + repulsive_force[1])

            net_force_magnitude = (net_force[0]**2 + net_force[1]**2)**0.5
            if net_force_magnitude != 0:
                net_force_unit = (net_force[0] / net_force_magnitude, net_force[1] / net_force_magnitude)
            else:
                net_force_unit = (0, 0)
            line_length = 50  # You can adjust this length
            net_heading_point = (int(center[0] + net_force_unit[0] * line_length), int(center[1] + net_force_unit[1] * line_length))
            cv2.line(debug_image, center, net_heading_point, (0, 255, 0), 2)  # Use a distinct color like green

            net_heading = calculate_heading(center, (center[0] + net_force[0], center[1] + net_force[1]))
            heading_error = heading - net_heading 
            # Normalize the heading error to the range [-180, 180] for rotational control
            heading_error = (heading_error + 180) % 360 - 180

            # Check if the heading error is within the dead zone
            if abs(heading_error) <= dead_zone:
                # Inside the dead zone, make no rotation adjustment
                rotation_adjustment = 0
            else:
                # Outside the dead zone, calculate the rotation speed adjustment using proportional control
                rotation_adjustment = kp * heading_error

            # Determine the base speed for forward movement
            # This could be dynamically adjusted based on the distance to the goal or other criteria
            base_speed = max_speed
           
            # Determine the influence level of the net force
            force_influence_threshold = 0.5 * max_speed  # You can adjust this threshold based on your system's needs

            if robot_status_dict[tag.tag_id] and net_force_magnitude > force_influence_threshold:
                # When external force is significant, override distance-based calculations
                adjusted_speed = net_force_magnitude  # Directly use force magnitude to set speed
                adjusted_kp = 0.5 * kp  # Optionally adjust kp to make the robot more responsive to direction changes
            else:
                # Adjust speed and Kp based on distance to goal
                if distance_to_goal > close_threshold:
                    # Far from the goal, use normal behavior, max sensor range
                    adjusted_speed = base_speed
                    adjusted_kp = kp
                    dynamic_sensor_range = max_sensor_range
                elif distance_to_goal > stop_threshold:
                    # Close to the goal, reduce speed and Kp
                    adjusted_speed = max(min_speed, base_speed * (distance_to_goal / close_threshold))  # Linear scaling
                    adjusted_kp = kp * (distance_to_goal / close_threshold)  # Linear scaling
                    dynamic_sensor_range = max(min_sensor_range, max_sensor_range * (distance_to_goal / close_threshold))
                else:
                    # Very close to the goal, prepare to stop or move minimally
                    adjusted_speed = 0
                    adjusted_kp = 0
                    dynamic_sensor_range = min_sensor_range

            cv2.circle(debug_image, center, int(dynamic_sensor_range/2), (255,255,255), 0) #Draw circle goal location

            # Recalculate rotation_adjustment with adjusted_kp
            rotation_adjustment = adjusted_kp * heading_error

            
            proximity_sensors = elisa.getAllProximity(tag.tag_id)

            threshold_proximity = 25  # Define your threshold value here

            # Adjust the left and right wheel speeds based on the adjusted rotation adjustment and speed
            left_speed = adjusted_speed - rotation_adjustment
            right_speed = adjusted_speed + rotation_adjustment

            # Apply the calculated speeds to the robot, ensuring they do not exceed maximum capabilities
            left_speed = max(min(left_speed, max_speed), -max_speed)
            right_speed = max(min(right_speed, max_speed), -max_speed)
                
            elisa.setLeftSpeed(tag.tag_id, int(left_speed))
            elisa.setRightSpeed(tag.tag_id, int(right_speed))

            # Stop the robot if it is within a close distance to the goal
            if distance_to_goal <= stop_threshold:
                robot_status_dict[tag.tag_id] = True  # Mark as arrived

    # print("-----------")
    # Place the formation switch check after processing all tags
    all_arrived = all(robot_status_dict.values())
    
    #Code to print batery from robots
    count = 0
    for i in robotAddr:
        print(str(robotAddr[count]) + " battery = " + str(elisa.getBatteryPercent(robotAddr[count])))
        count+=1
    print("---------------------------")
    
    if all_arrived:
        formation_id += 1
        if formation_id < len(formations):
            current_formation = formations[formation_id]
            display_locations = current_formation.copy()
            processed_tags.clear()  # Reset processed tags for the new formation
            robot_status_dict = {robot_id: False for robot_id in robotAddr}  # Reset status for the new formation
            assigned_goals_bool = False
        else:
            print("Completed all formations.")
            quit()
            break  # Or any other action you'd like to perform after completing all formations

    for i in display_locations:
        cv2.circle(debug_image, i, 4, (0,255,255), -1) #Draw circle goal location
    
    # for robot_id in robotAddr:
    #     if robot_id not in detected_robot_ids:
    #         # This robot was not detected, so set its speed to 0
    #         elisa.setLeftSpeed(robot_id, 0)
    #         elisa.setRightSpeed(robot_id, 0)
    #         # if robot_id in robot_dict:
    #         #     del robot_dict[str(robot_id)] # Optional: Remove it from robot_dict if you're tracking goal positions

    cv2.imshow("IMG", debug_image)
            
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break

    time.sleep(0.1)

   
