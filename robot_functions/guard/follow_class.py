#!/usr/bin/python3
# coding=utf8

import os
import sys
import cv2
import time
import math
import threading
import numpy as np
import pandas as pd
'''
from HiwonderSDK.PID import PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.Camera as Camera
import HiwonderSDK.ActionGroupControl as AGC
from CameraCalibration.CalibrationConfig import *
'''
import robot_functions.guard.stubs.HiwonderSDK.Board as Board
import robot_functions.guard.stubs.HiwonderSDK.Camera as Camera
import robot_functions.guard.stubs.HiwonderSDK.ActionGroupControl as AGC
import robot_functions.guard.stubs.HiwonderSDK.Misc as Misc
from robot_functions.guard.stubs.CameraCalibration.CalibrationConfig import calibration_param_path
from robot_functions.guard.stubs.HiwonderSDK.PID import PID

import HiwonderSDK.yaml_handle as yaml_handle


class FollowController:
    def __init__(self, required_detect_seconds=15.0, debug=False, use_hsv=True):
        self.debug = debug
        self.required_detect_seconds = required_detect_seconds
        self.use_hsv = use_hsv
        self.target_caught = False

        if sys.version_info.major == 2:
            print('Please run this program with python3!')
            sys.exit(0)

        # Load camera calibration parameters
        param_data = np.load(calibration_param_path + '.npz')
        self.mtx = param_data['mtx_array']
        self.dist = param_data['dist_array']
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.mtx, self.dist, (640, 480), 0, (640, 480)
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.mtx, self.dist, None, self.newcameramtx, (640, 480), 5
        )

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.lab_data = None
        self.lab_data_hsv = None
        self.servo_data = None
        self.follow_thread = None

        self.target_color = ('red',)
        self.is_running = False

        self.d_x = 20
        self.d_y = 20
        self.step = 1
        self.start_count = True

        self.centerX, self.centerY = -2, -2
        self.CENTER_X = 320
        self.circle_radius = 0

        self.x_pid = PID(P=0.4, I=0.02, D=0.02)
        self.y_pid = PID(P=0.4, I=0.02, D=0.02)

        self.x_dis = 1500
        self.y_dis = 1500

        self.radius_data = []
        self.size = (320, 240)
        self.last_hsv = None

        # Kept exactly as in your current file
        self.HEAD_TILT_SERVO_ID = 2
        self.HEAD_UP_PULSE = 1500
        self.MOVE_TIME_MS = 300

        self.load_config()
        self.reset()

    # Loads LAB/HSV color thresholds and servo defaults from yaml
    def load_config(self):
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
        self.lab_data_hsv = yaml_handle.get_yaml_data(yaml_handle.lab_hsv_file_path)
        self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

    # Set detection color
    def set_ball_target_color(self, target_color):
        self.target_color = target_color
        return (True, ())

    # Alias to keep old naming compatible
    def setBallTargetColor(self, target_color):
        return self.set_ball_target_color(target_color)

    # Moves the robot to its initial starting position
    def initMove(self):
        Board.setPWMServoPulse(1, self.servo_data['servo1'], 500)
        Board.setPWMServoPulse(2, self.servo_data['servo2'], 500)

    # Resets PID, robot positions, and detection state
    def reset(self):
        self.d_x = 20
        self.d_y = 20
        self.step = 1
        self.x_pid.clear()
        self.y_pid.clear()
        self.x_dis = self.servo_data['servo2']
        self.y_dis = self.servo_data['servo1']
        self.start_count = True
        self.target_color = ('red',)
        self.centerX, self.centerY = -2, -2
        self.circle_radius = 0
        self.radius_data = []

    # App initialization call
    def init(self):
        print("Follow Init")
        self.load_config()
        self.initMove()

    # Helper for guard_main.py
    def look_up(self):
        Board.setPWMServoPulse(self.HEAD_TILT_SERVO_ID, self.HEAD_UP_PULSE, self.MOVE_TIME_MS)
        time.sleep(self.MOVE_TIME_MS / 1000.0)

    # Helper for guard_main.py
    def target_visible(self):
        return self.centerX >= 0

    # Helper for guard_main.py
    def get_center_yaw(self):
        return self.servo_data['servo2']

    # Enables tracking
    def start(self):
        self.reset()
        self.is_running = True

        if self.follow_thread is None or not self.follow_thread.is_alive():
            self.follow_thread = threading.Thread(target=self._move_loop)
            self.follow_thread.daemon = True
            self.follow_thread.start()

        print("Follow Start")

    # Disables tracking
    def stop(self):
        self.is_running = False
        print("Follow Stop")

    # Stops game and runs a stand action group
    def exit(self):
        self.is_running = False
        AGC.runActionGroup('stand_slow')
        print("Follow Exit")

    # Find the contour with the largest area
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp >= 100:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    # Background action loop
    def _move_loop(self):
        Board.setPWMServoPulse(self.HEAD_TILT_SERVO_ID, self.HEAD_UP_PULSE, self.MOVE_TIME_MS)
        time.sleep(self.MOVE_TIME_MS / 1000.0)
        detect_start = None

        while True:
            if not self.is_running:
                detect_start = None
                time.sleep(0.01)
                continue

            if self.centerX >= 0:
                AGC.runActionGroup('stand')

                if detect_start is None:
                    detect_start = time.monotonic()

                if (time.monotonic() - detect_start) >= self.required_detect_seconds:
                    # AGC.runActionGroup('twist')
                    self.target_caught = True
                    print("GUARD IS A WINNER")
                    return True

                CENTER_DEADZONE = 150
                HEAD_DEADZONE = 120

                if self.centerX - self.CENTER_X > CENTER_DEADZONE or self.x_dis - self.servo_data['servo2'] < -HEAD_DEADZONE:
                    AGC.runActionGroup('turn_right_small_step')
                elif self.centerX - self.CENTER_X < -CENTER_DEADZONE or self.x_dis - self.servo_data['servo2'] > HEAD_DEADZONE:
                    AGC.runActionGroup('turn_left_small_step')
                elif self.circle_radius > 0:
                    AGC.runActionGroup('go_forward')
                    AGC.runActionGroup('go_forward')
                    AGC.runActionGroup('go_forward')
                    AGC.runActionGroup('go_forward')
                    AGC.runActionGroup('go_forward')
                    AGC.runActionGroup('go_forward')
                else:
                    time.sleep(0.01)
            else:
                detect_start = None
                time.sleep(0.01)

    # LAB version
    def run_lab(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        if not self.is_running or self.target_color == ():
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0

        for i in self.lab_data:
            if i in self.target_color:
                frame_mask = cv2.inRange(
                    frame_lab,
                    (
                        self.lab_data[i]['min'][0],
                        self.lab_data[i]['min'][1],
                        self.lab_data[i]['min'][2]
                    ),
                    (
                        self.lab_data[i]['max'][0],
                        self.lab_data[i]['max'][1],
                        self.lab_data[i]['max'][2]
                    )
                )
                cv2.imshow('mask', frame_mask)
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)

        if areaMaxContour is not None and area_max > 100:
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            for j in range(4):
                box[j, 0] = int(Misc.map(box[j, 0], 0, self.size[0], 0, img_w))
                box[j, 1] = int(Misc.map(box[j, 1], 0, self.size[1], 0, img_h))

            cv2.drawContours(img, [box], -1, (0, 255, 255), 2)

            ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            radius = abs(ptime_start_x - pt3_x)

            self.centerX = int((ptime_start_x + pt3_x) / 2)
            self.centerY = int((ptime_start_y + pt3_y) / 2)

            print('CenterX =', self.centerX, 'centerY =', self.centerY, 'circle_radius =', self.circle_radius)
            cv2.circle(img, (self.centerX, self.centerY), 5, (0, 255, 255), -1)

            use_time = 0

            self.radius_data.append(radius)
            data = pd.DataFrame(self.radius_data)
            data_ = data.copy()
            u = data_.mean()
            std = data_.std()

            data_c = data[np.abs(data - u) <= std]
            self.circle_radius = round(data_c.mean()[0], 1)

            if len(self.radius_data) == 5:
                self.radius_data.remove(self.radius_data[0])

            self.x_pid.SetPoint = img_w / 2
            self.x_pid.update(self.centerX)
            dx = int(self.x_pid.output)
            use_time = abs(dx * 0.00025)
            self.x_dis += dx

            self.x_dis = self.servo_data['servo2'] - 400 if self.x_dis < self.servo_data['servo2'] - 400 else self.x_dis
            self.x_dis = self.servo_data['servo2'] + 400 if self.x_dis > self.servo_data['servo2'] + 400 else self.x_dis

            self.y_pid.SetPoint = img_h / 2
            self.y_pid.update(self.centerY)
            dy = int(self.y_pid.output)
            use_time = round(max(use_time, abs(dy * 0.00025)), 5)
            self.y_dis += dy

            self.y_dis = self.servo_data['servo1'] if self.y_dis < self.servo_data['servo1'] else self.y_dis
            self.y_dis = 2000 if self.y_dis > 2000 else self.y_dis

            Board.setPWMServoPulse(1, self.y_dis, use_time * 1000)
            Board.setPWMServoPulse(2, self.x_dis, use_time * 1000)
            time.sleep(use_time)
        else:
            self.centerX, self.centerY = -1, -1

        return img

    # HSV version
    def run_hsv(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        if not self.is_running or self.target_color == ():
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_hsv = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2HSV)
        self.last_hsv = frame_hsv

        area_max = 0
        areaMaxContour = None
        merged_mask = np.zeros(frame_hsv.shape[:2], dtype=np.uint8)

        for i in self.lab_data_hsv:
            if i in self.target_color:
                mask = cv2.inRange(
                    frame_hsv,
                    tuple(self.lab_data_hsv[i]['min']),
                    tuple(self.lab_data_hsv[i]['max'])
                )
                merged_mask = cv2.bitwise_or(merged_mask, mask)

        cv2.imshow('mask', merged_mask)
        cv2.imshow("masked", cv2.bitwise_and(frame_resize, frame_resize, mask=merged_mask))

        eroded = cv2.erode(merged_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areaMaxContour, area_max = self.getAreaMaxContour(contours)

        if areaMaxContour is not None and area_max > 100:
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            for j in range(4):
                box[j, 0] = int(Misc.map(box[j, 0], 0, self.size[0], 0, img_w))
                box[j, 1] = int(Misc.map(box[j, 1], 0, self.size[1], 0, img_h))

            cv2.drawContours(img, [box], -1, (0, 255, 255), 2)

            ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            radius = abs(ptime_start_x - pt3_x)

            self.centerX = int((ptime_start_x + pt3_x) / 2)
            self.centerY = int((ptime_start_y + pt3_y) / 2)

            print('CenterX =', self.centerX, 'centerY =', self.centerY, 'circle_radius =', self.circle_radius)
            cv2.circle(img, (self.centerX, self.centerY), 5, (0, 255, 255), -1)

            self.radius_data.append(radius)
            data = pd.DataFrame(self.radius_data)
            data_ = data.copy()
            u = data_.mean()
            std = data_.std()
            data_c = data[np.abs(data - u) <= std]
            self.circle_radius = round(data_c.mean()[0], 1)

            if len(self.radius_data) == 5:
                self.radius_data.remove(self.radius_data[0])

            self.x_pid.SetPoint = img_w / 2
            self.x_pid.update(self.centerX)
            dx = int(self.x_pid.output)
            use_time = abs(dx * 0.00025)
            self.x_dis += dx

            self.x_dis = self.servo_data['servo2'] - 400 if self.x_dis < self.servo_data['servo2'] - 400 else self.x_dis
            self.x_dis = self.servo_data['servo2'] + 400 if self.x_dis > self.servo_data['servo2'] + 400 else self.x_dis

            self.y_pid.SetPoint = img_h / 2
            self.y_pid.update(self.centerY)
            dy = int(self.y_pid.output)
            use_time = round(max(use_time, abs(dy * 0.00025)), 5)
            self.y_dis += dy

            self.y_dis = self.servo_data['servo1'] if self.y_dis < self.servo_data['servo1'] else self.y_dis
            self.y_dis = 2000 if self.y_dis > 2000 else self.y_dis

            Board.setPWMServoPulse(1, self.y_dis, use_time * 1000)
            Board.setPWMServoPulse(2, self.x_dis, use_time * 1000)
            time.sleep(use_time)
        else:
            self.centerX, self.centerY = -1, -1

        return img

    # Main run method for guard_main.py
    def run(self, img):
        if self.use_hsv:
            return self.run_hsv(img)
        return self.run_lab(img)

    def show_hsv(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE and param is not None:
            hsv = param[y, x]
            print("HSV:", hsv)


if __name__ == '__main__':
    follow = FollowController(use_hsv=True)

    follow.init()
    follow.start()
    follow.set_ball_target_color(('red',))
    print('target_color = ', follow.target_color)

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
            Frame = follow.run_hsv(frame)
            cv2.imshow('Frame', Frame)
            cv2.setMouseCallback("Frame", follow.show_hsv, follow.last_hsv)

            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)

    try:
        my_camera.camera_close()
    except Exception:
        try:
            my_camera.release()
        except Exception:
            pass

    cv2.destroyAllWindows()