#!/usr/bin/python3
# coding=utf8
"""
VisualPatrol rewritten as a reusable class so it can be controlled by guard_main.py.

Behavior kept the same:
- Patrols a floor line (default black)
- Starts by looking down at the floor
- Uses a background movement loop to walk based on line position
- Keeps helper methods like look_down_at_floor() and look_up_and_scan()
"""

import sys
import cv2
import time
import threading
import numpy as np
'''
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.Camera as Camera
import HiwonderSDK.ActionGroupControl as AGC
'''
import robot_functions.guard.stubs.HiwonderSDK.Board as Board
import robot_functions.guard.stubs.HiwonderSDK.Camera as Camera
import robot_functions.guard.stubs.HiwonderSDK.ActionGroupControl as AGC
import robot_functions.guard.stubs.HiwonderSDK.Misc as Misc


import HiwonderSDK.yaml_handle as yaml_handle




class VisualPatrolController:
    def __init__(self, floor_pitch=1000, floor_yaw=1500):
        if sys.version_info.major == 2:
            print('Please run this program with python3!')
            sys.exit(0)

        self.target_color = ('black',)
        self.lab_data = None
        self.servo_data = None
        self.is_running = False
        self.line_centerx = -1

        self.FLOOR_PITCH = floor_pitch
        self.FLOOR_YAW = floor_yaw

        self.roi = [
            (240, 280, 0, 640, 0.1),
            (340, 380, 0, 640, 0.3),
            (440, 480, 0, 640, 0.6)
        ]
        self.roi_h_list = [
            self.roi[0][0],
            self.roi[1][0] - self.roi[0][0],
            self.roi[2][0] - self.roi[1][0]
        ]
        self.size = (640, 480)

        self.move_thread = None

        self.load_config()

    def set_line_target_color(self, target_color):
        self.target_color = target_color
        return (True, (), 'SetVisualPatrolColor')

    # keep old naming style too
    def setLineTargetColor(self, target_color):
        return self.set_line_target_color(target_color)

    def load_config(self):
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
        self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

    def initMove(self):
        Board.setPWMServoPulse(1, self.servo_data['servo1'], 500)  # pitch
        Board.setPWMServoPulse(2, self.servo_data['servo2'], 500)  # yaw

    def look_down_at_floor(self):
        Board.setPWMServoPulse(1, self.FLOOR_PITCH, 500)
        Board.setPWMServoPulse(2, self.FLOOR_YAW, 500)
        time.sleep(0.5)

    def look_up_and_scan(self):
        Board.setPWMServoPulse(1, 1500, 500)
        time.sleep(0.5)
        for pos in range(self.servo_data['servo2'] - 350, self.servo_data['servo2'] + 351, 75):
            Board.setPWMServoPulse(2, pos, 400)
            time.sleep(0.35)
        self.look_down_at_floor()

    def reset(self):
        self.line_centerx = -1
        self.target_color = ()

    def init(self):
        print("VisualPatrol Init")
        self.load_config()
        self.initMove()

    def start(self):
        self.reset()
        self.look_down_at_floor()
        self.is_running = True

        if self.move_thread is None or not self.move_thread.is_alive():
            self.move_thread = threading.Thread(target=self._move_loop)
            self.move_thread.daemon = True
            self.move_thread.start()

        print("VisualPatrol Start")

    def stop(self):
        self.is_running = False
        print("VisualPatrol Stop")

    def exit(self):
        self.is_running = False
        try:
            AGC.runActionGroup('stand_low')
        except Exception:
            pass
        try:
            Board.setPWMServoPulse(1, self.FLOOR_PITCH, 500)
            Board.setPWMServoPulse(2, self.FLOOR_YAW, 500)
        except Exception:
            pass
        print("VisualPatrol Exit")

    def getAreaMaxContour(self, contours):
        contour_area_max = 0
        area_max_contour = None
        for c in contours:
            area = abs(cv2.contourArea(c))
            if area > contour_area_max:
                contour_area_max = area
                if area >= 5:
                    area_max_contour = c
        return area_max_contour, contour_area_max

    def _move_loop(self):
        img_centerx = 320
        while True:
            if self.is_running:
                if self.line_centerx != -1:
                    diff = self.line_centerx - img_centerx
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

    def run(self, img):
        if not self.is_running or self.target_color == ():
            return img

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

        centroid_x_sum = 0
        weight_sum = 0
        center_y = 0

        for n, r in enumerate(self.roi):
            roi_h = self.roi_h_list[n]
            blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)

            dilated = None
            for colour in self.lab_data:
                if colour in self.target_color:
                    frame_mask = cv2.inRange(
                        frame_lab,
                        (
                            self.lab_data[colour]['min'][0],
                            self.lab_data[colour]['min'][1],
                            self.lab_data[colour]['min'][2]
                        ),
                        (
                            self.lab_data[colour]['max'][0],
                            self.lab_data[colour]['max'][1],
                            self.lab_data[colour]['max'][2]
                        )
                    )
                    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                    this_dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                    dilated = this_dilated if dilated is None else cv2.bitwise_or(dilated, this_dilated)

            if dilated is None:
                continue

            dilated[:, 0:160] = 0
            dilated[:, 480:640] = 0

            cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            cnt_large, area = self.getAreaMaxContour(cnts)

            if cnt_large is not None:
                rect = cv2.minAreaRect(cnt_large)
                box = np.int0(cv2.boxPoints(rect))

                for i in range(4):
                    box[i, 1] = box[i, 1] + n * roi_h + self.roi[0][0]
                    box[i, 1] = int(Misc.map(box[i, 1], 0, self.size[1], 0, img_h))
                for i in range(4):
                    box[i, 0] = int(Misc.map(box[i, 0], 0, self.size[0], 0, img_w))

                cv2.drawContours(img, [box], -1, (0, 0, 255), 2)

                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x = (pt1_x + pt3_x) / 2
                center_y = (pt1_y + pt3_y) / 2
                cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]

        if weight_sum != 0:
            self.line_centerx = int(centroid_x_sum / weight_sum)
            cv2.circle(img, (self.line_centerx, int(center_y)), 10, (0, 255, 255), -1)
        else:
            self.line_centerx = -1

        return img


if __name__ == '__main__':
    patrol = VisualPatrolController()

    patrol.init()
    patrol.start()
    patrol.set_line_target_color(('black',))

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

    try:
        patrol.look_down_at_floor()
    except Exception:
        pass

    try:
        while True:
            ret, img = my_camera.read()
            if ret:
                frame = img.copy()
                Frame = patrol.run(frame)
                cv2.imshow('result_image', Frame)

            key = cv2.waitKey(1)
            if key == 27:
                break

            time.sleep(0.01)

    finally:
        try:
            if open_once:
                my_camera.release()
            else:
                my_camera.camera_close()
        except Exception:
            pass
        cv2.destroyAllWindows()