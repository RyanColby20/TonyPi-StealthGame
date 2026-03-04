#!/usr/bin/python3
# coding=utf8
"""
Customised version of the VisualPatrol script for the TonyPi/HiWonder robot.

This version:
- removes the OpenCV mouse callback dependency,
- follows a floor line (default: black),
- periodically scans for a red intruder,
- keeps the video feed alive during scanning,
- and, if red is detected, hands off to Follow.py.

Important handoff detail:
The stock Hiwonder Follow.py usually reads its startup head pose from
servo_config.yaml. To make Follow.py begin from the same pose where the red
object was detected, this script writes that detected head pose into the servo
config file immediately before launching Follow.py.
"""

import sys
import cv2
import time
import threading
import subprocess
import numpy as np

import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

# -----------------------------------------------------------------------------
# Python version check
# -----------------------------------------------------------------------------
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# -----------------------------------------------------------------------------
# Global state
# -----------------------------------------------------------------------------
__target_color = ('black',)
lab_data = None
servo_data = None
__isRunning = False
line_centerx = -1

# Camera undistortion maps (filled in main)
mapx = None
mapy = None

# Pose where the intruder was seen
last_detected_pitch = None
last_detected_yaw = None

# Regions of interest for line following
roi = [
    (240, 280, 0, 640, 0.1),
    (340, 380, 0, 640, 0.3),
    (440, 480, 0, 640, 0.6)
]
roi_h_list = [roi[0][0], roi[1][0] - roi[0][0], roi[2][0] - roi[1][0]]
size = (640, 480)


def setLineTargetColor(target_color):
    """Set the target line colour as a tuple of colour names."""
    global __target_color
    __target_color = target_color
    return (True, (), 'SetVisualPatrolColor')


def load_config():
    """Load LAB colour thresholds and servo positions from YAML files."""
    global lab_data, servo_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


def initMove():
    """Move the head servos to their configured default positions."""
    Board.setPWMServoPulse(1, servo_data['servo1'], 500)  # pitch
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)  # yaw


def reset():
    """Reset runtime state before patrol starts."""
    global line_centerx, __target_color
    line_centerx = -1
    __target_color = ()


def init():
    print("VisualPatrol Init")
    load_config()
    initMove()


def start():
    """Start patrol mode."""
    global __isRunning
    reset()
    look_up_and_scan()  # initial dramatic sweep
    __isRunning = True
    print("VisualPatrol Start")


def stop():
    global __isRunning
    __isRunning = False
    print("VisualPatrol Stop")


def exit():
    global __isRunning
    __isRunning = False
    try:
        AGC.runActionGroup('stand_low')
    except Exception:
        pass
    print("VisualPatrol Exit")


def save_follow_start_pose(pitch, yaw):
    """Persist a startup pose so stock Follow.py starts from this same head pose."""
    try:
        with open(yaml_handle.servo_file_path, 'w', encoding='utf-8') as f:
            f.write(f"servo1: {int(pitch)}\nservo2: {int(yaw)}\n")
        servo_data['servo1'] = int(pitch)
        servo_data['servo2'] = int(yaw)
        return True
    except Exception as e:
        print(f"Could not save Follow.py start pose: {e}")
        return False


def look_up_and_scan():
    """Initial sweep: look up, scan wider than before, then return to default."""
    Board.setPWMServoPulse(1, 1500, 500)
    time.sleep(0.5)
    # Wider scan than before
    for pos in range(servo_data['servo2'] - 350, servo_data['servo2'] + 351, 75):
        Board.setPWMServoPulse(2, pos, 400)
        time.sleep(0.35)
    Board.setPWMServoPulse(1, servo_data['servo1'], 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)


def getAreaMaxContour(contours):
    """Return the largest contour and its area."""
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        area = abs(cv2.contourArea(c))
        if area > contour_area_max:
            contour_area_max = area
            if area >= 5:
                area_max_contour = c
    return area_max_contour, contour_area_max


def move():
    """Background thread: walk according to the line position."""
    global line_centerx
    img_centerx = 320
    while True:
        if __isRunning:
            if line_centerx != -1:
                diff = line_centerx - img_centerx
                if abs(diff) <= 50:
                    AGC.runActionGroup('go_forward')
                elif diff > 50:
                    AGC.runActionGroup('turn_right_small_step')
                else:
                    AGC.runActionGroup('turn_left_small_step')
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)


# Launch movement thread
th = threading.Thread(target=move)
th.daemon = True
th.start()


def run(img):
    """Process one frame to find the line and update line_centerx."""
    global line_centerx, __target_color

    if not __isRunning or __target_color == ():
        return img

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

    centroid_x_sum = 0
    weight_sum = 0
    center_y = 0

    for n, r in enumerate(roi):
        roi_h = roi_h_list[n]
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)

        dilated = None
        for colour in lab_data:
            if colour in __target_color:
                frame_mask = cv2.inRange(
                    frame_lab,
                    (lab_data[colour]['min'][0], lab_data[colour]['min'][1], lab_data[colour]['min'][2]),
                    (lab_data[colour]['max'][0], lab_data[colour]['max'][1], lab_data[colour]['max'][2])
                )
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                this_dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = this_dilated if dilated is None else cv2.bitwise_or(dilated, this_dilated)

        if dilated is None:
            continue

        # ignore noisy far-left / far-right areas
        dilated[:, 0:160] = 0
        dilated[:, 480:640] = 0

        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
        cnt_large, area = getAreaMaxContour(cnts)
        if cnt_large is not None:
            rect = cv2.minAreaRect(cnt_large)
            box = np.int0(cv2.boxPoints(rect))

            for i in range(4):
                box[i, 1] = box[i, 1] + n * roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0, 0, 255), 2)

            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x = (pt1_x + pt3_x) / 2
            center_y = (pt1_y + pt3_y) / 2
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if weight_sum != 0:
        line_centerx = int(centroid_x_sum / weight_sum)
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0, 255, 255), -1)
    else:
        line_centerx = -1

    return img


def follow_intruder(my_camera, start_pitch, start_yaw):
    """Stop patrol and launch Follow.py from the pose where red was detected."""
    global __isRunning
    __isRunning = False

    # Hold the head exactly where the intruder was seen.
    try:
        Board.setPWMServoPulse(1, int(start_pitch), 150)
        Board.setPWMServoPulse(2, int(start_yaw), 150)
    except Exception:
        pass

    # Make Follow.py inherit this as its startup pose.
    save_follow_start_pose(start_pitch, start_yaw)

    # Close current camera if needed
    try:
        if hasattr(my_camera, 'camera_close'):
            my_camera.camera_close()
    except Exception:
        pass

    # Launch Follow.py
    try:
        subprocess.Popen(['python3', 'Follow.py'])
        print('Launching Follow.py to track the intruder...')
    except FileNotFoundError:
        print('Follow.py not found. Put it in the same folder or update the path.')

    cv2.destroyAllWindows()
    sys.exit(0)


def scan_for_intruder(my_camera):
    """Pause patrol, scan slowly for red, then either resume or hand off."""
    global __isRunning, last_detected_pitch, last_detected_yaw

    __isRunning = False

    # Look up before scanning
    scan_pitch = 1500
    Board.setPWMServoPulse(1, scan_pitch, 500)
    time.sleep(0.5)

    intruder_found = False

    # Much wider, slower, more dramatic scan
    for pos in range(servo_data['servo2'] - 450, servo_data['servo2'] + 451, 80):
        Board.setPWMServoPulse(2, pos, 250)
        time.sleep(0.1)

        ret, frame = my_camera.read()
        if not ret or frame is None:
            continue

        if mapx is not None and mapy is not None:
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        # Keep laptop preview updating during scan so the feed does not appear frozen.
        cv2.imshow('result_image', frame)
        cv2.waitKey(1)

        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        red_min = tuple(lab_data['red']['min'])
        red_max = tuple(lab_data['red']['max'])
        mask = cv2.inRange(frame_lab, red_min, red_max)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour, area = getAreaMaxContour(contours)

        if contour is not None and area > 200:
            print("INTRUDER DETECTED")
            intruder_found = True
            last_detected_pitch = scan_pitch
            last_detected_yaw = pos
            follow_intruder(my_camera, last_detected_pitch, last_detected_yaw)
            break

    if not intruder_found:
        # Return to patrol head pose and resume.
        Board.setPWMServoPulse(1, servo_data['servo1'], 500)
        Board.setPWMServoPulse(2, servo_data['servo2'], 500)
        time.sleep(0.5)
        __isRunning = True

    return intruder_found


if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import calibration_param_path

    # Camera calibration
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    init()
    start()
    setLineTargetColor(('black',))

    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()

    try:
        AGC.runActionGroup('stand')
    except Exception:
        pass

    last_scan_time = time.time()
    scan_interval = 5.0

    try:
        while True:
            ret, img = my_camera.read()
            if ret:
                frame = img.copy()
                frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
                Frame = run(frame)
                cv2.imshow('result_image', Frame)

                if time.time() - last_scan_time > scan_interval:
                    scan_for_intruder(my_camera)
                    last_scan_time = time.time()

            key = cv2.waitKey(1)
            if key == 27:
                break

            time.sleep(0.01)

    finally:
        try:
            if not open_once:
                my_camera.camera_close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        exit()

