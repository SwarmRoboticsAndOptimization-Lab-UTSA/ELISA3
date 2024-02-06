import elisa3
import time
import cv2
from pupil_apriltags import Detector
import copy
import subprocess
from utils import *

commands = [
    "v4l2-ctl --set-ctrl=auto_exposure=1",
    "v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_time_absolute=120",
    "v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=25",
    "v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=0"
    
]

for command in commands:
    try:
        subprocess.check_call(command, shell=True)
    except subprocess.CalledProcessError:
        print(f"Error executing the command: {command}")

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
desired_location = [(282,96),(414,960),(546,96),(813,96),(480,120),(480,205),(480,290),(480,375),(480,548)] #Goal Locations
display_locations = desired_location.copy()
robot_dict = {}
 
robotAddr = [3948]
elisa = elisa3.Elisa3(robotAddr)
elisa.start()

counter = 0

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
        min_distance = float('inf')
        mid = midpoint(corner_01,corner_02)
        cv2.line(debug_image, (center[0], center[1]),(mid[0], mid[1]), (255, 255, 0), 2)
        heading = calculate_heading(center,mid)

        if tag_identifier not in processed_tags:
            # Assuming desired_location is defined and find_and_remove_closest_point as previously described
            modified_list, closest_point = find_and_remove_closest_point(list(desired_location), center)
            processed_tags.add(tag_identifier)  # Mark the tag as processed
            robot_dict[str(tag.tag_id)] = closest_point #Add robot and its desired location

        if tag.tag_id in robotAddr:
            desired_heading = calculate_heading(center,robot_dict[str(tag.tag_id)])

            rotation_direction = calculate_rotation_direction(heading,desired_heading)
            if rotation_direction == "no rotation":
                elisa.setLeftSpeed(tag.tag_id, 5)
                elisa.setRightSpeed(tag.tag_id, 5)

            elif rotation_direction == "left":
                elisa.setLeftSpeed(tag.tag_id, -5)
                elisa.setRightSpeed(tag.tag_id, 5)
                
            elif rotation_direction == "right":
                elisa.setLeftSpeed(tag.tag_id, 5)
                elisa.setRightSpeed(tag.tag_id, -5)

            distance_to_goal = calculate_distance(mid[0],mid[1],robot_dict[str(tag.tag_id)][0],robot_dict[str(tag.tag_id)][1])

            if distance_to_goal <= 10:
                elisa.setLeftSpeed(tag.tag_id, 0)
                elisa.setRightSpeed(tag.tag_id, 0)            

    for i in display_locations:
        cv2.circle(debug_image, i, 10, (0,255,255), -1) #Draw circle goal location
    
    
    cv2.imshow("IMG", debug_image)
            
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        des_speed_right = 0
        des_speed_left = 0 #500 FORWARD 400 TURN LEFT WHILE MOVING FORWARD
        cap.release()
        cv2.destroyAllWindows()
        break


    counter = counter + 1
    time.sleep(0.1)

   
