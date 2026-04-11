#!/usr/bin/python3
# coding=utf8
"""
This code is the same code as Follow.py but refactored to fit within an Object Oriented paradigm.

Note: it has not been tested to work on the robots. 

This code was originally taken and refactored for OOP from Follow.py located in the Github on April 10th, 2026
"""

#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np

try:
    from hiwonder.PID import PID
    import hiwonder.Misc as Misc
    import hiwonder.Board as Board
    import hiwonder.ActionGroupControl as AGC
    import hiwonder.yaml_handle as yaml_handle
except ImportError:
    from HiwonderSDK.PID import PID
    import HiwonderSDK.Misc as Misc
    import HiwonderSDK.Board as Board
    import HiwonderSDK.ActionGroupControl as AGC
    import HiwonderSDK.yaml_handle as yaml_handle


class Follow_cls:
    """
    Intruder follow behavior for guard_main.

    Public API:
        start(initial_head_pose=None)
        stop()
        shutdown()
        update(dt) -> "continue" | "intruder_captured" | "lost_intruder"
    """

    CENTER_X = 320

    def __init__(
        self,
        robot_controller,
        target_color=('red',),
        capture_seconds=10.0,
        lost_timeout=1.0,
    ):
        self.robot = robot_controller
        self.target_color = target_color
        self.capture_seconds = capture_seconds
        self.lost_timeout = lost_timeout

        self.lab_data = None
        self.servo_data = None
        self.mapx = None
        self.mapy = None

        self.x_pid = PID(P=0.4, I=0.02, D=0.02)
        self.y_pid = PID(P=0.4, I=0.02, D=0.02)

        self.size = (320, 240)
        self.radius_data = []

        self.running = False
        self.start_time = 0.0
        self.last_seen_time = 0.0
        self.visible_since = None

        self.x_dis = 1500
        self.y_dis = 1500
        self.centerX = -1
        self.centerY = -1
        self.circle_radius = 0

        self._load_config()
        self._load_calibration()
        self.reset()

        self._shutdown_event = threading.Event()
        self._move_thread = threading.Thread(target=self._move_loop, daemon=True)
        self._move_thread.start()

    # ------------------------------------------------------------------
    # setup / teardown
    # ------------------------------------------------------------------
    def _load_config(self):
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
        self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

    def _load_calibration(self):
        try:
            from CameraCalibration.CalibrationConfig import calibration_param_path
        except ImportError:
            try:
                from Functions.CameraCalibration.CalibrationConfig import calibration_param_path
            except ImportError:
                calibration_param_path = None

        if calibration_param_path is None:
            return

        try:
            param_data = np.load(calibration_param_path + '.npz')
            mtx = param_data['mtx_array']
            dist = param_data['dist_array']
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                mtx, dist, None, newcameramtx, (640, 480), 5
            )
        except Exception:
            self.mapx = None
            self.mapy = None

    def reset(self):
        self.x_pid.clear()
        self.y_pid.clear()

        self.x_dis = int(self.servo_data['servo2'])
        self.y_dis = int(self.servo_data['servo1'])

        self.centerX = -1
        self.centerY = -1
        self.circle_radius = 0
        self.radius_data = []

        self.visible_since = None
        self.start_time = 0.0
        self.last_seen_time = 0.0

    def start(self, initial_head_pose=None):
        self.reset()
        self.running = True
        self.start_time = time.time()
        self.last_seen_time = self.start_time

        if initial_head_pose is not None:
            pitch, yaw = initial_head_pose
            self.y_dis = int(pitch)
            self.x_dis = int(yaw)

        try:
            Board.setPWMServoPulse(1, int(self.y_dis), 250)
            Board.setPWMServoPulse(2, int(self.x_dis), 250)
        except Exception:
            pass

        return "continue"

    def stop(self):
        self.running = False
        self.centerX = -1
        self.centerY = -1
        self.circle_radius = 0

    def shutdown(self):
        self.stop()
        self._shutdown_event.set()

    # ------------------------------------------------------------------
    # camera / frame helpers
    # ------------------------------------------------------------------
    def _get_frame(self):
        for name in ("get_frame", "get_latest_frame", "read_camera"):
            fn = getattr(self.robot, name, None)
            if callable(fn):
                value = fn()
                if isinstance(value, tuple) and len(value) >= 2:
                    ret, frame = value[0], value[1]
                    return frame if ret else None
                return value

        camera = getattr(self.robot, "camera", None)
        if camera is not None and hasattr(camera, "read"):
            value = camera.read()
            if isinstance(value, tuple) and len(value) >= 2:
                ret, frame = value[0], value[1]
                return frame if ret else None
            return value

        return None

    def _prepare_frame(self, frame):
        if frame is None:
            return None
        if self.mapx is not None and self.mapy is not None:
            return cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        return frame

    # ------------------------------------------------------------------
    # vision helpers
    # ------------------------------------------------------------------
    def _get_area_max_contour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for contour in contours:
            contour_area_temp = math.fabs(cv2.contourArea(contour))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp >= 100:
                    area_max_contour = contour

        return area_max_contour, contour_area_max

    def _update_radius(self, radius):
        self.radius_data.append(radius)
        if len(self.radius_data) > 5:
            self.radius_data.pop(0)

        values = np.array(self.radius_data, dtype=float)
        if len(values) == 1:
            self.circle_radius = float(values[0])
            return

        mean = values.mean()
        std = values.std()
        if std == 0:
            filtered = values
        else:
            filtered = values[np.abs(values - mean) <= std]

        if len(filtered) == 0:
            filtered = values

        self.circle_radius = float(np.round(filtered.mean(), 1))

    def _process_target(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        area_max_contour = None

        for color_name in self.target_color:
            if color_name not in self.lab_data:
                continue

            frame_mask = cv2.inRange(
                frame_lab,
                tuple(self.lab_data[color_name]['min']),
                tuple(self.lab_data[color_name]['max']),
            )
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contour, contour_area = self._get_area_max_contour(contours)
            if contour is not None and contour_area > area_max:
                area_max = contour_area
                area_max_contour = contour

        if area_max_contour is None or area_max <= 100:
            self.centerX, self.centerY = -1, -1
            return False

        rect = cv2.minAreaRect(area_max_contour)
        box = cv2.boxPoints(rect).astype(int)

        for i in range(4):
            box[i, 0] = int(Misc.map(box[i, 0], 0, self.size[0], 0, img_w))
            box[i, 1] = int(Misc.map(box[i, 1], 0, self.size[1], 0, img_h))

        pt1_x, pt1_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]
        radius = abs(pt1_x - pt3_x)

        self.centerX = int((pt1_x + pt3_x) / 2)
        self.centerY = int((pt1_y + pt3_y) / 2)
        self._update_radius(radius)

        self.x_pid.SetPoint = img_w / 2
        self.x_pid.update(self.centerX)
        dx = int(self.x_pid.output)
        use_time = abs(dx * 0.00025)
        self.x_dis += dx

        default_yaw = int(self.servo_data['servo2'])
        self.x_dis = max(default_yaw - 400, min(default_yaw + 400, self.x_dis))

        self.y_pid.SetPoint = img_h / 2
        self.y_pid.update(self.centerY)
        dy = int(self.y_pid.output)
        use_time = round(max(use_time, abs(dy * 0.00025)), 5)
        self.y_dis += dy

        default_pitch = int(self.servo_data['servo1'])
        self.y_dis = max(default_pitch, min(2000, self.y_dis))

        try:
            Board.setPWMServoPulse(1, int(self.y_dis), int(use_time * 1000))
            Board.setPWMServoPulse(2, int(self.x_dis), int(use_time * 1000))
        except Exception:
            pass

        if use_time > 0:
            time.sleep(use_time)

        return True

    # ------------------------------------------------------------------
    # behavior
    # ------------------------------------------------------------------
    def _move_loop(self):
        while not self._shutdown_event.is_set():
            if not self.running:
                time.sleep(0.01)
                continue

            if self.centerX >= 0:
                default_yaw = int(self.servo_data['servo2'])

                try:
                    if self.centerX - self.CENTER_X > 100 or self.x_dis - default_yaw < -80:
                        AGC.runActionGroup('turn_right_small_step')
                    elif self.centerX - self.CENTER_X < -100 or self.x_dis - default_yaw > 80:
                        AGC.runActionGroup('turn_left_small_step')
                    elif 0 < self.circle_radius < 100:
                        AGC.runActionGroup('go_forward')
                    elif self.circle_radius > 180:
                        AGC.runActionGroup('back_fast')
                    else:
                        time.sleep(0.01)
                except Exception:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def update(self, dt):
        if not self.running:
            return "continue"

        frame = self._prepare_frame(self._get_frame())
        if frame is None:
            return "continue"

        detected = self._process_target(frame)
        now = time.time()

        if detected:
            self.last_seen_time = now
            if self.visible_since is None:
                self.visible_since = now
            elif now - self.visible_since >= self.capture_seconds:
                return "intruder_captured"
        else:
            self.visible_since = None
            if now - self.last_seen_time > self.lost_timeout:
                return "lost_intruder"

        return "continue"