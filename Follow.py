#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np
import pandas as pd
from hiwonder.PID import PID
import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
import time
from CameraCalibration.CalibrationConfig import *

# Follow

debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Load parameters
param_data = np.load(calibration_param_path + '.npz')

# Get parameters
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

__target_color = ('green',)
# Set detection color
def setBallTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# Initial position
def initMove():
    Board.setPWMServoPulse(1, servo_data['servo1'], 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)
    Board.setPWMServoPulse(1,1500,500)

load_config()

d_x = 20
d_y = 20
step = 1
x_dis = servo_data['servo2']
y_dis = servo_data['servo1']
start_count = True
centerX, centerY = -2, -2
x_pid = PID(P=0.4, I=0.02, D=0.02) # PID initialization
y_pid = PID(P=0.4, I=0.02, D=0.02)
# Reset variables
def reset():
    global d_x, d_y
    global start_count
    global step, step_
    global x_dis, y_dis
    global __target_color
    global centerX, centerY

    d_x = 20
    d_y = 20
    step = 1
    x_pid.clear()
    y_pid.clear()
    x_dis = servo_data['servo2']
    y_dis = servo_data['servo1']
    start_count = True
    __target_color = ()
    centerX, centerY = -2, -2
    
# App initialization call
def init():
    print("Follow Init")
    load_config()
    initMove()

__isRunning = False
# App start-game call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Follow Start")

# App stop-game call
def stop():
    global __isRunning
    __isRunning = False
    print("Follow Stop")

# App exit-game call
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("Follow Exit")

# Find the contour with the largest area 
# Parameter: list of contours to compare
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Iterate through all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 100:  # Only contours above this threshold are valid to filter noise
                area_max_contour = c

    return area_max_contour, contour_area_max  # Return the largest contour

CENTER_X = 320
circle_radius = 0

# Execute action group
import time
HEAD_TILT_SERVO_ID = 2
HEAD_UP_PULSE = 2000
MOVE_TIME_MS = 300

def move(required_detect_seconds=15.0):
    
    Board.setPWMServoPulse(HEAD_TILT_SERVO_ID, HEAD_UP_PULSE, MOVE_TIME_MS)
    time.sleep(MOVE_TIME_MS / 1000)
    detect_start = None  # when uninterrupted detection began

    while True:
        if not __isRunning:
            detect_start = None  # treat as interruption
            time.sleep(0.01)
            continue

        # --- Detection logic ---
        if centerX >= 0:
            AGC.runActionGroup('stand')
            # start uninterrupted timer if we just regained detection
            if detect_start is None:
                detect_start = time.monotonic()

            # if uninterrupted detection has lasted long enough, stop/return
            if (time.monotonic() - detect_start) >= required_detect_seconds:
                AGC.runActionGroup('twist')
                print("GUARD IS A WINNER")
                # Optional "stop" action if your robot has one:
                # AGC.runActionGroup('stand') or AGC.stopActionGroup() depending on your SDK
                return True

            # --- your existing movement logic (unchanged) ---
            if centerX - CENTER_X > 100 or x_dis - servo_data['servo2'] < -80:
                AGC.runActionGroup('turn_right_small_step')
            elif centerX - CENTER_X < -100 or x_dis - servo_data['servo2'] > 80:
                AGC.runActionGroup('turn_left_small_step')
            elif 100 > circle_radius > 0:
                AGC.runActionGroup('go_forward')
            elif 180 < circle_radius:
                AGC.runActionGroup('back_fast')
            else:
                time.sleep(0.01)

        else:
            # lost detection → reset timer
            detect_start = None
            time.sleep(0.01)

# Start the action thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

radius_data = []
size = (320, 240)
def run(img):
    global radius_data
    global x_dis, y_dis
    global centerX, centerY, circle_radius
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __target_color == ():
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB color space
    
    area_max = 0
    areaMaxContour = 0
    for i in lab_data:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab,
                                     (lab_data[i]['min'][0],
                                      lab_data[i]['min'][1],
                                      lab_data[i]['min'][2]),
                                     (lab_data[i]['max'][0],
                                      lab_data[i]['max'][1],
                                      lab_data[i]['max'][2]))  # Bitwise mask
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # Erode
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) # Dilate
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find contours
            areaMaxContour, area_max = getAreaMaxContour(contours)  # Find largest contour
    if areaMaxContour is not None and area_max > 100:  # Largest valid contour found
        rect = cv2.minAreaRect(areaMaxContour) # Minimum bounding rectangle
        box = np.int0(cv2.boxPoints(rect)) # Four corner points of the rectangle
        for j in range(4):
            box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

        cv2.drawContours(img, [box], -1, (0,255,255), 2) # Draw rectangle
        # Get diagonal points
        ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]
        radius = abs(ptime_start_x - pt3_x)
        centerX, centerY = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2) # Center point       
        cv2.circle(img, (centerX, centerY), 5, (0, 255, 255), -1) # Draw center point
          
        use_time = 0       
        
        radius_data.append(radius)
        data = pd.DataFrame(radius_data)
        data_ = data.copy()
        u = data_.mean()  # Compute mean
        std = data_.std()  # Compute standard deviation

        data_c = data[np.abs(data - u) <= std]
        circle_radius = round(data_c.mean()[0], 1)
        if len(radius_data) == 5:
            radius_data.remove(radius_data[0])
            
        #print(circle_radius)
        x_pid.SetPoint = img_w/2 # Setpoint           
        x_pid.update(centerX) # Current value
        dx = int(x_pid.output)
        use_time = abs(dx*0.00025)
        x_dis += dx # Output           
        
        x_dis = servo_data['servo2'] - 400 if x_dis < servo_data['servo2'] - 400 else x_dis          
        x_dis = servo_data['servo2'] + 400 if x_dis > servo_data['servo2'] + 400 else x_dis
            
        y_pid.SetPoint = img_h/2
        y_pid.update(centerY)
        dy = int(y_pid.output)
        use_time = round(max(use_time, abs(dy*0.00025)), 5)
        y_dis += dy
        
        y_dis = servo_data['servo1'] if y_dis < servo_data['servo1'] else y_dis
        y_dis = 2000 if y_dis > 2000 else y_dis    
        
        Board.setPWMServoPulse(1, y_dis, use_time*1000)
        Board.setPWMServoPulse(2, x_dis, use_time*1000)
        time.sleep(use_time)
    else:
        centerX, centerY = -1, -1
   
    #img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)   # Distortion correction 

    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red',)
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open() 
    AGC.runActionGroup('stand')
    while True:
        ret, img = my_camera.read()
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()

