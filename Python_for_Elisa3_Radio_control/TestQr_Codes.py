import cv2
from pupil_apriltags import Detector
import subprocess
from utils import midpoint,get_formations_list

def set_camera_setting(setting, value):
    try:
        command = f"v4l2-ctl --device=/dev/video0 --set-ctrl={setting}={value}"
        subprocess.check_call(command, shell=True)
        subprocess.check_call("v4l2-ctl --set-ctrl=auto_exposure=1", shell=True)
        subprocess.check_call("v4l2-ctl --set-ctrl=focus_absolute=1", shell=True)
        

    except subprocess.CalledProcessError:
        print(f"Error executing the command: {command}")

def on_trackbar_change_exposure(value):
    # Exposure might need to convert this value into a correct range your camera supports
    set_camera_setting("exposure_time_absolute", value)

def on_trackbar_change_contrast(value):
    set_camera_setting("contrast", value)

def on_trackbar_change_brightness(value):
    set_camera_setting("brightness", value)

def on_trackbar_change_zoom(value):
    set_camera_setting("zoom_absolute", value)

def toggle_auto_focus(value):
    # Convert the incoming value to a boolean or use directly if already boolean
    set_camera_setting("focus_absolute", value)


at_detector = Detector(
        families="tagCustom48h12",
        nthreads=1,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )


cv2.namedWindow('Frame')
cv2.createTrackbar('Exposure', 'Frame', 0, 2047, on_trackbar_change_exposure)
cv2.createTrackbar('Contrast', 'Frame', 0, 255, on_trackbar_change_contrast)
cv2.createTrackbar('Brightness', 'Frame', 0, 255, on_trackbar_change_brightness)
cv2.createTrackbar('Zoom', 'Frame', 100, 500, on_trackbar_change_zoom)  # Adjust the range according to your camera
cv2.createTrackbar('Auto Focus', 'Frame', 0, 250, toggle_auto_focus)

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        image,
        estimate_tag_pose=False,
        camera_params=None,
        tag_size=None,
    )    


    display_locations = get_formations_list()
    # for i in display_locations[3]:
    #     cv2.circle(frame, i, 4, (0,255,255), -1) #Draw circle goal location

    for tag in tags:
        center = (int(tag.center[0]), int(tag.center[1]))
        corners = tag.corners
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        mid = midpoint(corner_01,corner_02)
        cv2.line(frame, (center[0], center[1]),(mid[0], mid[1]), (255, 255, 0), 2)
        cv2.putText(frame, str(tag.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()