#!/usr/bin/python3
# coding=utf8
"""
This is the refactored version of VisualPatrol made to fit within OOP,

Note: it has not been tested to work on the robots.

This code was originally taken and refactored for OOP from VisualPatrol.py located in the Github on April 10th, 2026
"""
#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import threading
import numpy as np

try:
    import hiwonder.Misc as Misc
    import hiwonder.Board as Board
    import hiwonder.ActionGroupControl as AGC
    import hiwonder.yaml_handle as yaml_handle
except ImportError:
    import HiwonderSDK.Misc as Misc
    import HiwonderSDK.Board as Board
    import HiwonderSDK.ActionGroupControl as AGC
    import HiwonderSDK.yaml_handle as yaml_handle


class Visual_Patrol_cls:
    """
    Line-follow patrol behavior for guard_main.

    Public API:
        start()
        stop()
        shutdown()
        update(dt) -> "continue" | "intruder_detected"
        get_last_detection_pose() -> (pitch, yaw)
    """

    def __init__(
        self,
        robot_controller,
        line_color=('black',),
        intruder_color=('red',),
        floor_pitch=1000,
        floor_yaw=1500,
        scan_interval=5.0,
    ):
        self.robot = robot_controller

        self.line_color = line_color
        self.intruder_color = intruder_color

        self.floor_pitch = floor_pitch
        self.floor_yaw = floor_yaw
        self.scan_interval = scan_interval

        self.lab_data = None
        self.servo_data = None

        self.running = False
        self.line_centerx = -1
        self.last_scan_time = 0.0

        self.last_detected_pitch = 1500
        self.last_detected_yaw = 1500

        self.size = (640, 480)
        self.roi = [
            (240, 280, 0, 640, 0.1),
            (340, 380, 0, 640, 0.3),
            (440, 480, 0, 640, 0.6),
        ]
        self.roi_h_list = [
            self.roi[0][0],
            self.roi[1][0] - self.roi[0][0],
            self.roi[2][0] - self.roi[1][0],
        ]

        self.mapx = None
        self.mapy = None
        self._load_config()
        self._load_calibration()

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

    def start(self):
        self.line_centerx = -1
        self.running = True
        self.last_scan_time = time.time()
        self.look_down_at_floor()
        return "continue"

    def stop(self):
        self.running = False
        self.line_centerx = -1

    def shutdown(self):
        self.stop()
        self._shutdown_event.set()
        try:
            self.look_down_at_floor()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # head helpers
    # ------------------------------------------------------------------
    def look_down_at_floor(self):
        Board.setPWMServoPulse(1, int(self.floor_pitch), 500)
        Board.setPWMServoPulse(2, int(self.floor_yaw), 500)
        time.sleep(0.35)

    def get_last_detection_pose(self):
        return int(self.last_detected_pitch), int(self.last_detected_yaw)

    # ------------------------------------------------------------------
    # camera / frame helpers
    # ------------------------------------------------------------------
    def _get_frame(self):
        # Preferred: robot controller owns the shared camera
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
        contour_area_max = 0
        area_max_contour = None

        for contour in contours:
            area = abs(cv2.contourArea(contour))
            if area > contour_area_max:
                contour_area_max = area
                if area >= 5:
                    area_max_contour = contour

        return area_max_contour, contour_area_max

    def _red_visible(self, frame, min_area=200):
        if frame is None or self.lab_data is None or 'red' not in self.lab_data:
            return False

        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        red_min = tuple(self.lab_data['red']['min'])
        red_max = tuple(self.lab_data['red']['max'])

        mask = cv2.inRange(frame_lab, red_min, red_max)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour, area = self._get_area_max_contour(contours)
        return contour is not None and area > min_area

    def _process_line(self, img):
        if img is None:
            self.line_centerx = -1
            return

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

        centroid_x_sum = 0.0
        weight_sum = 0.0

        for idx, region in enumerate(self.roi):
            roi_h = self.roi_h_list[idx]
            blobs = frame_gb[region[0]:region[1], region[2]:region[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)

            dilated = None
            for color_name in self.line_color:
                if color_name not in self.lab_data:
                    continue

                frame_mask = cv2.inRange(
                    frame_lab,
                    tuple(self.lab_data[color_name]['min']),
                    tuple(self.lab_data[color_name]['max']),
                )
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                this_dilated = cv2.dilate(
                    eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                )
                dilated = this_dilated if dilated is None else cv2.bitwise_or(dilated, this_dilated)

            if dilated is None:
                continue

            dilated[:, 0:160] = 0
            dilated[:, 480:640] = 0

            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            contour, area = self._get_area_max_contour(contours)
            if contour is None or area <= 0:
                continue

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect).astype(int)

            for i in range(4):
                box[i, 1] = box[i, 1] + idx * roi_h + self.roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, self.size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, self.size[0], 0, img_w))

            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x = (pt1_x + pt3_x) / 2.0

            centroid_x_sum += center_x * region[4]
            weight_sum += region[4]

        if weight_sum != 0:
            self.line_centerx = int(centroid_x_sum / weight_sum)
        else:
            self.line_centerx = -1

    # ------------------------------------------------------------------
    # behavior
    # ------------------------------------------------------------------
    def _move_loop(self):
        img_centerx = 320

        while not self._shutdown_event.is_set():
            if self.running and self.line_centerx != -1:
                diff = self.line_centerx - img_centerx
                try:
                    if abs(diff) <= 50:
                        AGC.runActionGroup('go_forward')
                    elif diff > 50:
                        AGC.runActionGroup('turn_right_small_step')
                    else:
                        AGC.runActionGroup('turn_left_small_step')
                except Exception:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def _scan_for_intruder(self):
        self.running = False
        self.line_centerx = -1

        scan_pitch = 1500
        Board.setPWMServoPulse(1, scan_pitch, 500)
        time.sleep(0.35)

        center_yaw = int(self.servo_data['servo2'])
        found_intruder = False

        for yaw in range(center_yaw - 450, center_yaw + 451, 80):
            Board.setPWMServoPulse(2, int(yaw), 250)
            time.sleep(0.10)

            frame = self._prepare_frame(self._get_frame())
            if frame is None:
                continue

            if self._red_visible(frame):
                self.last_detected_pitch = scan_pitch
                self.last_detected_yaw = int(yaw)
                found_intruder = True
                break

        if not found_intruder:
            self.look_down_at_floor()
            self.running = True

        return found_intruder

    def update(self, dt):
        if not self.running:
            return "continue"

        frame = self._prepare_frame(self._get_frame())
        if frame is None:
            return "continue"

        self._process_line(frame)

        if time.time() - self.last_scan_time >= self.scan_interval:
            self.last_scan_time = time.time()
            if self._scan_for_intruder():
                return "intruder_detected"

        return "continue"