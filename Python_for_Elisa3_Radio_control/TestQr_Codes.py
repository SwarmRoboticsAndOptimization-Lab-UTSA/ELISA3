import cv2
from pupil_apriltags import Detector
import subprocess

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

    for tag in tags:
        center = (int(tag.center[0]), int(tag.center[1]))

        cv2.putText(frame, str(tag.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()