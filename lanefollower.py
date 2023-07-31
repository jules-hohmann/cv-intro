import cv2
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import sys
import signal
import random as rand
from motor_test import test_motor
import time
from pymavlink import mavutil
IP_ADDRESS = "10.29.77.218"
vcap = cv2.VideoCapture(f"rtsp://{IP_ADDRESS}:8554/rovcam")

img = None

#cap = cv2.VideoCapture('AUV_Vid.mkv')

#ret, frame = cap.read()

def common_member(a, b):
    a_set = set(a)
    b_set = set(b)
    if (a_set & b_set):
        return True
    else:
        return False

def detect_lines(img, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100,maxLineGap=20):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize) # detect edges
    lines = cv2.HoughLinesP(edges,10,np.pi/210,70,minLineLength=270,maxLineGap=25) # detect lines
    if lines is not None:
        return((lines))
    else:
        #raise(ValueError)
        return(None)
    
def drawlane(img, lines, colors):
    
    line = 0
    if lines is None:
            pass
    else:
        while line < len(lines):
            x1 = lines[line][0]
            x2 = lines[line][1]
            y1 = lines[line][2]
            y2 = lines[line][3]
            cv2.line(img, (x1, y1), (x2, y2), colors, 3)
            line+=1
        
    return(img)
   
def draw_lines(img, lines, colors):
    if lines is None:
            pass
    else:
        for line in lines:
            
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), colors, 5)
        
    return(img)

def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    line = 0
    while line < len(lines):
        
        slope = (lines[line][0][3] - lines[line][0][2]) / (lines[line][0][1] - lines[line][0][0])
        intercept = lines[line][0][2] - slope * lines[line][0][0]
        slopes.append(slope)
        intercepts.append(intercept)
        line+=1
    return ((slopes, intercepts))

#Rome did this
def get_color():
    c=rand.randint(0,255)
    b=rand.randint(0,255)
    a=rand.randint(0,255)
    return((a,b,c))

def detect_lanes(lines):
    lanes = []

    if lines is None:
        return None
    else:
        slopes, intercepts = get_slopes_intercepts(lines)
        for i in range(len(lines)):
            if lines[i][0][3] > len(img):
                pass
            else:
                for j in range(i+1, len(lines)):
                    
                    if (np.absolute(lines[i][0][0]-lines[j][0][0]) < 40) and (np.absolute(lines[i][0][1]-lines[j][0][1]) < 40):
                        j+=1
                        break
                    if np.absolute(np.abs(slopes[i] - slopes[j])) < 0.25:
                        lanes.append([lines[i], lines[j]])
                        i += 1 #to make sure single line isn't paired with more than one other line

    return (np.array(lanes).tolist()) 

def draw_lanes(image, lanes):
    if lanes is None:
        pass
    else:
        for lane in lanes:
            color = get_color()
            draw_lines(image, lane, color)
    return image

def get_lane_center(lanes):
    
    if lanes is None:
        pass
    else:
        a = lanes[0][0]
        b = lanes[0][1]
        c = (a[0][3] - a[0][2]) / (a[0][1] - a[0][0])
        d = (b[0][3] - b[0][2]) / (b[0][1] - b[0][0])
        slope = (c+d)/ 2
        center = (a[0][2] - c * b[0][0]) + (b[0][2]-d*b[0][0]) / 2
        for i in range(0, len(lanes)-1):
            lane_center = (get_slopes_intercepts(lanes[i])[0][0] + get_slopes_intercepts(lanes[i+1])[0][1]) / 2
            # we get intercepts for line 1 and line 2 for each lane and get the average
            if np.abs(lane_center - len(img[1])/2) < np.abs(center - len(img[0])/2):
                center = lane_center
                print((center, slope))
        return ((center, slope))
##Rome wrote this
def recommend_direction(center):
    if center is None:
        pass
    else:
        #Gets if center is within 10 pixels of 960, it returns forward, otherwise gets back to center
        if center< len(img[0]-10):
            return("left")
        elif center>len(img[0]+10):
            return("right")
        elif center <970 and center>950:
            return("forward")

def recommend_turn(slope):

    if np.abs(slope) > 3:
        return "don't turn"
    elif slope > -3 and slope < 0:
        return "turn right"
    else:
        return "turn left"
    
def recommend_movement(center, slope):
    return recommend_direction(center), recommend_turn(slope)


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        # time.sleep(0.2)
        step += 0.2

def forward(t, m):
    run_motors_timed(mav_connection, seconds=t, motor_settings=[m, m, -m, -m, 0, 0])
         
def rightstrafe(t, m):
         run_motors_timed(mav_connection, seconds = t, motor_settings= [m, -m, m, -m])
def leftstrafe(t, m):
        run_motors_timed(mav_connection, seconds=t, motor_settings= [ -m, m, -m, m])

def leftturn(t,m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[-m, m, m, -m, 0, 0])
def rightturn(t, m):
        run_motors_timed(mav_connection, seconds=t, motor_settings=[m, -m, -m, m, 0, 0])

def movetowardlane(direction, turning):
    
    if direction == "right":
        rightstrafe(3, 30)
    if direction == "left":
        leftstrafe(3, 30)
    if direction == "forward":
        forward(3, 30)
    if turning == "turn left":
        leftturn(3, 30)
    if turning == "turn right":
        rightturn(3, 30)

if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    MAX_RETRY = 100
    count = 0
    while count < MAX_RETRY:
        count += 1
        # Obtain the frame
        ret, frame = vcap.read()

        # Check frame was received successfully
        if ret:
            # got a frame, close the cap and return the frame
            print(" got a frame ")
            img = frame
            a=detect_lines(img, 36, 80, 3)
            b = detect_lanes(a)
            plt.imshow(draw_lanes(img, b))
            plt.show()
            if b == None or len(b) == 0:
                print("no recomended direction")
            else:
                lane_center = get_lane_center(b)
                direction = recommend_direction(lane_center[0])
                turning = recommend_turn(lane_center[1])
                movetowardlane(direction, turning)
            vcap.release()

    vcap.release()