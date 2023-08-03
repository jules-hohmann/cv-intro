import matplotlib.pyplot as plt
import random as rand
from motor_test import test_motor
from time import sleep
from pymavlink import mavutil
import something
import PID as pid
from threading import Thread, Event
import lanedection as ld
from bluerov_interface import BlueROV

video = something.Video()
pid_horizontal = pid(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

longitudinal_power = 0
lateral_power = 0

#cap = cv2.VideoCapture('AUV_Vid.mkv')

#ret, frame = cap.read()



def _get_frame():
    global frame
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            if video.frame_available():
                frame = video.frame()
                img = frame
                a=ld.detect_lines(img, 36, 80, 3)
                b = ld.detect_lanes(a)
                plt.imshow(ld.draw_lanes(img, b))
                plt.show()
                if b == None or len(b) == 0:
                    print("no recomended direction")
                    longitudinal_power = 0.1
                    lateral_power = 0
                else:
                    lane_center = ld.get_lane_center(b)
                    direction = ld.recommend_direction(lane_center[0], img)
                    turning = ld.recommend_turn(lane_center[1])
                    if direction == "right":
                        lateral_power = ld.get_distance_from_lane(lane_center, img)/100
                        longitudinal_power = 0.2
                    elif direction == "left":
                        lateral_power = -ld.get_distance_from_lane(lane_center, img)/100
                        longitudinal_power = 0.2
                    elif direction == "forward":
                        longitudinal_power = 0.3
                    print(frame.shape)

    except KeyboardInterrupt:
        return

def _send_rc():
    bluerov.set_lateral_power(lateral_power)
    bluerov.set_longitudinal_power(longitudinal_power)

# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.disarm()
    print("Exiting...")


