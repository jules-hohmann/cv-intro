
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import random as rand

def common_member(a, b):
    a_set = set(a)
    b_set = set(b)
    if (a_set & b_set):
        return True
    else:
        return False

def detect_lines(img, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100,maxLineGap=20):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    blur = cv2.medianBlur(gray, 31)
    edges = cv2.Canny(blur, threshold1, threshold2, apertureSize) # detect edges
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

def detect_lanes(lines, img):
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

def get_lane_center(lanes, img):
    
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
def recommend_direction(center, img):
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

def get_distance_from_lane(lane_center = [0, 0], img = 5):
    if img == 5:
        return 0
    else:
        img_center = len(img)[1]/2
        d = np.abs(img-lane_center[0])
        return d

