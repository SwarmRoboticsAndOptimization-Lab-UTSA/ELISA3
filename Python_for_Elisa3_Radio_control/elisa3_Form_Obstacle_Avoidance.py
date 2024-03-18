import elisa3
import time
import cv2
from pupil_apriltags import Detector
import copy
import subprocess
from utils import *


try:
    subprocess.check_call("v4l2-ctl --set-ctrl=auto_exposure=1", shell=True)
    subprocess.check_call("v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_time_absolute=34", shell=True)
    subprocess.check_call("v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=15", shell=True)
    subprocess.check_call("v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=39", shell=True)
    subprocess.check_call("v4l2-ctl --device=/dev/video0 --set-ctrl=zoom_absolute=148", shell=True)
    subprocess.check_call("v4l2-ctl --device=/dev/video0 --set-ctrl=focus_automatic_continuous=0", shell=True)
    subprocess.check_call("v4l2-ctl --device=/dev/video0 --set-ctrl=focus_absolute=0", shell=True)
    
except subprocess.CalledProcessError:
    print(f"Error executing the command:")

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
formation_id = 0
formations = get_formations_list()
current_formation = formations[formation_id]
display_locations = current_formation.copy()
robot_dict = {}
robotAddr = [3948,4060,4021,4469,3988,3846,4050,4096,4101,4083]
robot_status_dict = {robot_id: False for robot_id in robotAddr} #Status of location of the robots
elisa = elisa3.Elisa3(robotAddr)
elisa.start()

counter = 0
speed = 0 #Speed of robots


while True:
    ret, frame = cap.read()
    if not ret:
        break
    debug_image = copy.deepcopy(frame)

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        image,
        estimate_tag_pose=False,
        camera_params=None,
        tag_size=None,
    )


    for tag in tags:
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
        heading = calculate_heading(center,mid)

        if tag_identifier not in processed_tags:
            # Assuming desired_location is defined and find_and_remove_closest_point as previously described
            current_formation, closest_point = find_and_remove_closest_point(list(current_formation), center)
            processed_tags.add(tag_identifier)  # Mark the tag as processed
            robot_dict[str(tag.tag_id)] = closest_point #Add robot and its desired location
      
        # Initialize repulsive force for current robot
        repulsive_force = (0, 0)

        # Calculate repulsive force from all other robots
        for other_tag in tags:
            if tag.tag_id != other_tag.tag_id:
                other_center = (int(other_tag.center[0]), int(other_tag.center[1]))
                obstacle_repulsion = calculate_repulsive_force(center, other_center, sensor_range=50, strength=2.0)
                repulsive_force = (repulsive_force[0] + obstacle_repulsion[0], repulsive_force[1] + obstacle_repulsion[1])

        # Calculate attractive force towards the goal
        if str(tag.tag_id) in robot_dict:
            goal_position = robot_dict[str(tag.tag_id)]
            attractive_force = calculate_attractive_force(center, goal_position, strength=1.0)

            # Calculate net force
            net_force = (attractive_force[0] + repulsive_force[0], attractive_force[1] + repulsive_force[1])

            # Determine movement direction based on net force
            net_heading = calculate_heading(center, (center[0] + net_force[0], center[1] + net_force[1]))
            rotation_direction = calculate_rotation_direction(heading, net_heading)

            prox = elisa.getAllProximity(tag.tag_id)
            if prox is not None:
                if prox[0] > 40:
                    elisa.setLeftSpeed(tag.tag_id, -3)
                    elisa.setRightSpeed(tag.tag_id, -3)
                else:
                    # Adjust robot movement based on the net force
                    if rotation_direction == "no rotation":
                        elisa.setLeftSpeed(tag.tag_id, speed)
                        elisa.setRightSpeed(tag.tag_id, speed)
                    elif rotation_direction == "left":
                        elisa.setLeftSpeed(tag.tag_id, -speed)
                        elisa.setRightSpeed(tag.tag_id, speed)
                    elif rotation_direction == "right":
                        elisa.setLeftSpeed(tag.tag_id, speed)
                        elisa.setRightSpeed(tag.tag_id, -speed)

                # Stop the robot if it is within a close distance to the goal
                distance_to_goal = calculate_distance(mid[0], mid[1], goal_position[0], goal_position[1])
                if distance_to_goal <= 5:
                    elisa.setLeftSpeed(tag.tag_id, 0)
                    elisa.setRightSpeed(tag.tag_id, 0)
                    robot_status_dict[tag.tag_id] = True
                    #print(robot_status_dict[tag.tag_id])

    # Place the formation switch check after processing all tags
    all_arrived = all(robot_status_dict.values())

    if all_arrived:
        formation_id += 1
        if formation_id < len(formations):
            current_formation = formations[formation_id]
            display_locations = current_formation.copy()
            processed_tags.clear()  # Reset processed tags for the new formation
            robot_status_dict = {robot_id: False for robot_id in robotAddr}  # Reset status for the new formation
        else:
            print("Completed all formations.")
            quit()
            break  # Or any other action you'd like to perform after completing all formations

    for i in display_locations:
        cv2.circle(debug_image, i, 4, (0,255,255), -1) #Draw circle goal location
    
    
    cv2.imshow("IMG", debug_image)
            
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break


    counter = counter + 1
    time.sleep(0.1)

   
