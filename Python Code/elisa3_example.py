import elisa3
import time
import cv2
from pupil_apriltags import Detector
import copy
import subprocess
from utils import *

commands = [
    "sudo v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_time_absolute=120",
    "sudo v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=25",
    "sudo v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=0"
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

#Robots Dictionary
robot_dic = {}
command_dict = {}
taken_locations = {}
desired_location = [[282,96],[414,96],[546,96],[813,96],[480,120],[480,205],[480,290],[480,375],[480,548]]
display_locations = desired_location.copy()

robotAddr = [4124]
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
        tag_family = tag.tag_family
        tag_id = tag.tag_id
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

        distances = [calculate_distance(mid[0],mid[1],des_loc[0],des_loc[1]) for des_loc in desired_location]
        if distances:
            min_distance = distances.index(min(distances)) #Get the index of the smallest distance.
            taken_locations[str(tag_id)] = desired_location[min_distance] #Use index of the smallest distance to update robot desired location
            desired_location.pop(min_distance)
        
        dist = calculate_distance(mid[0],mid[1],taken_locations[str(tag_id)][0],taken_locations[str(tag_id)][1])
        desired_heading = calculate_heading(center,taken_locations[str(tag_id)])
        robot_dic[str(tag_id)] = [heading,desired_heading, dist]

        print(robot_dic)
        elisa.setLeftSpeed(robotAddr[0], 0)
        elisa.setRightSpeed(robotAddr[0], 0)

        # if robot_dic:
        #     c_ind = 0
        #     # print(robot_dic)
        #     for id in robotAddr:
        #         rotation_direction = calculate_rotation_direction(robot_dic[id][0],robot_dic[id][1])

        #         if rotation_direction == "no rotation":
        #             des_speed_left = 100
        #             des_speed_right = 100

        #         elif rotation_direction == "left":
        #             des_speed_left = -100
        #             des_speed_right = 100
                
        #         elif rotation_direction == "right":
        #             des_speed_left = 100
        #             des_speed_right = -100

        #         distance_to_goal = robot_dic[id][2]
        #         if distance_to_goal <= 10:
        #             des_speed_left = 0
        #             des_speed_right = 0

        #         command_dict[str(c_ind)] = [des_speed_left, des_speed_right,id]
        #         c_ind +=1
                
        #         print(command_dict)
                
    for i in display_locations:
        cv2.circle(debug_image, i, 10, (0,255,255), -1) #Draw circle goal location
    
    cv2.imshow("IMG", debug_image)
            
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        des_speed_right = 0
        des_speed_left = 0 #500 FORWARD 400 TURN LEFT WHILE MOVING FORWARD
        break

    # elisa.setLeftSpeed(robotAddr[0], -10)
    # elisa.setRightSpeed(robotAddr[0], -10)
    # elisa.turnOffFrontIRs(robotAddr[0])
    # elisa.turnOnBackIR(robotAddr[0])        
    # elisa.setSmallLed(robotAddr[0], 0, 0)
    # elisa.setSmallLed(robotAddr[0], 1, 0)
    # elisa.setSmallLed(robotAddr[0], 2, 0)
    # elisa.setSmallLed(robotAddr[0], 3, 1)
    # elisa.setSmallLed(robotAddr[0], 4, 0)
    # elisa.setSmallLed(robotAddr[0], 5, 0)
    # elisa.setSmallLed(robotAddr[0], 6, 0)
    # elisa.setSmallLed(robotAddr[0], 7, 1)          
    # print(str(robotAddr[0]) + " battery = " + str(elisa.getBatteryPercent(robotAddr[0])))
    # print(str(robotAddr[0]) + " selector = " + str(elisa.getSelector(robotAddr[0])))        
        

    # prox = elisa.getAllProximity(robotAddr[0])
    # print(str(robotAddr[0]) + " prox: " + str(prox))

    # ground = elisa.getAllGround(robotAddr[0])
    # print(str(robotAddr[0]) + " ground: " + str(ground))

    # accx = elisa.getAccX(robotAddr[0])
    # accy = elisa.getAccY(robotAddr[0])
    # accz = elisa.getAccZ(robotAddr[0])
    # print(str(robotAddr[0]) + " acc: " + str(accx) + ", " + str(accy) + ", " + str(accz))

    # print(str(robotAddr[0]) + " theta: " + str(elisa.getOdomTheta(robotAddr[0])))
    # print(str(robotAddr[0]) + " xpos: " + str(elisa.getOdomXpos(robotAddr[0])))
    # print(str(robotAddr[0]) + " ypos: " + str(elisa.getOdomYpos(robotAddr[0])))
    # print(str(robotAddr[0]) + " left steps: " + str(elisa.getLeftMotSteps(robotAddr[0])))
    # print(str(robotAddr[0]) + " right steps: " + str(elisa.getRightMotSteps(robotAddr[0])))
    

    counter = counter + 1
    time.sleep(0.1)