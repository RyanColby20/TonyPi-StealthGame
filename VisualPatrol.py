#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import math
import threading
import numpy as np

import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
from hiwonder.common import ColorPicker
from CameraCalibration.CalibrationConfig import *

"""
程序功能：视觉巡线(program function: vision line following)

运行效果：将机器人放置在黑线上，程序启动后，机器人会沿着黑线轨迹前进

此脚本是原 TonyPi 项目中 VisualPatrol.py 的简化版，用于演示如何使用本仓库提供的配置、动作组和控制库运行视觉巡线功能。
"""

# 确保使用 Python 3
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

target_color = []
board = rrc.Board()
ctl = Controller(board)
servo_data = None

# 加载配置文件数据
def load_config():
    global servo_data
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 初始化机器人舵机初始位置
def initMove():
    ctl.set_pwm_servo_pulse(1, servo_data['servo1'], 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)

line_center_x = -1

# 变量重置
def reset():
    global line_center_x
    global target_color
    global color_picker
    line_center_x = -1
    target_color = []
    color_picker = None

# 应用初始化调用
def init():
    global enter
    print("VisualPatrol Init")
    load_config()
    initMove()
    enter = True

enter = False
running = False

# 应用开始调用
def start():
    global running
    running = True
    print("VisualPatrol Start")

# 应用停止调用
def stop():
    global running
    running = False
    reset()
    print("VisualPatrol Stop")

# 应用退出调用
def exit():
    global enter, running
    enter = False
    running = False
    reset()
    AGC.runActionGroup('stand_slow')
    print("VisualPatrol Exit")

# 鼠标点击设置目标点
color_picker = None
def set_point(point):
    global color_picker, target_color
    x, y = point
    target_color = []
    color_picker = ColorPicker([x, y], 20)

# 获取当前选取的 RGB 值
def get_rgb_value():
    if target_color:
        return target_color[1]
    else:
        return []

# 颜色阈值参数
threshold = 0.3
def set_threshold(value):
    global threshold
    threshold = value

# 找出面积最大的轮廓
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 5:
                area_max_contour = c
    return area_max_contour, contour_area_max

# 动作控制线程
def move():
    global line_center_x
    while True:
        if enter:
            if line_center_x != -1:
                if abs(line_center_x - img_centerx) <= 50:
                    AGC.runActionGroup('go_forward')
                elif line_center_x - img_centerx > 50:
                    AGC.runActionGroup('turn_right_small_step')
                elif line_center_x - img_centerx < -50:
                    AGC.runActionGroup('turn_left_small_step')
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.1)

# 启动子线程
th = threading.Thread(target=move)
th.daemon = True
th.start()

# 设置感兴趣区域（ROI）
roi = [
        (240, 280, 0, 640, 0.1),
        (340, 380, 0, 640, 0.3),
        (440, 480, 0, 640, 0.6)
      ]
roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]
roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)
img_centerx = 320

def run(img):
    """处理一帧图像并根据识别结果更新 line_center_x"""
    global line_center_x
    global target_color
    global img_w, img_h
    global color_picker
    display_image = img.copy()
    img_h, img_w = img.shape[:2]
    if not enter:
        return display_image
    # 选择目标颜色
    if color_picker is not None and not target_color:
        target_color, display_image = color_picker(img, display_image)
        if target_color:
            color_picker = None
    elif target_color:
        # 缩放和滤波
        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        centroid_x_sum = 0
        weight_sum = 0
        center_ = []
        n = 0
        # 在每个 ROI 区域内寻找目标颜色最大轮廓
        for r in roi:
            roi_h = roi_h_list[n]
            n += 1
            blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)
            min_color = [int(target_color[0][0] - 50 * threshold * 2),
                         int(target_color[0][1] - 50 * threshold),
                         int(target_color[0][2] - 50 * threshold)]
            max_color = [int(target_color[0][0] + 50 * threshold * 2),
                         int(target_color[0][1] + 50 * threshold),
                         int(target_color[0][2] + 50 * threshold)]
            frame_mask = cv2.inRange(frame_lab, tuple(min_color), tuple(max_color))
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated[:, 0:160] = 0
            dilated[:, 480:640] = 0
            cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            cnt_large, area = getAreaMaxContour(cnts)
            if cnt_large is not None:
                rect = cv2.minAreaRect(cnt_large)
                box = np.int0(cv2.boxPoints(rect))
                for i in range(4):
                    box[i, 1] = box[i, 1] + (n - 1) * roi_h + roi[0][0]
                    box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
                for i in range(4):
                    box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
                cv2.drawContours(display_image, [box], -1, (0, 0, 255, 255), 2)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                cv2.circle(display_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                center_.append([center_x, center_y])
                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]
        if weight_sum != 0:
            line_center_x = int(centroid_x_sum / weight_sum)
            cv2.circle(display_image, (line_center_x, int(center_y)), 10, (0, 255, 255), -1)
        else:
            line_center_x = -1
    return display_image

if __name__ == '__main__':
    # 加载标定参数
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    init()
    start()
    # 开启相机
    try:
        open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml').get('open_once', False)
    except Exception:
        open_once = False
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()
    AGC.runActionGroup('stand')
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            Frame = run(frame)
            cv2.imshow('result_image', Frame)
            cv2.setMouseCallback("result_image", lambda event, x, y, flags, param: set_point((x/img_w, y/img_h)) if event == cv2.EVENT_LBUTTONDOWN else None)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()