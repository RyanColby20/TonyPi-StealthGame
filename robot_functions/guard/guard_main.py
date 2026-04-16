#!/usr/bin/python3
# coding=utf8

import sys
import cv2
import time
import numpy as np

import HiwonderSDK.Board as Board
import HiwonderSDK.Camera as Camera
import HiwonderSDK.ActionGroupControl as AGC
import HiwonderSDK.yaml_handle as yaml_handle


from Functions.CameraCalibration.CalibrationConfig import calibration_param_path
from robot_functions.guard.follow_class import FollowController
from robot_functions.guard.visualPatrol_class import VisualPatrolController

from robot_functions.game.robot_connection import RobotConnectionHandler
from robot_functions.robot_comm import RobotComm

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


# ----------------------------
# Timing / state settings
# ----------------------------
PATROL_SECONDS = 8.0
SEARCH_SECONDS = 5.0
LOST_TARGET_TIMEOUT = 1.5

STATE_PATROL = 'patrol'
STATE_SEARCH = 'search'


# ----------------------------
# Search pose settings
# ----------------------------
LOOK_UP_PULSE = 1500
HEAD_MOVE_MS = 300


# ----------------------------
# Search pan helper
# ----------------------------
class LeftCenterPan:
    def __init__(self, servo_id=2, move_time_ms=250, dwell_time=0.65, left_offset=500):
        self.servo_id = servo_id
        self.move_time_ms = move_time_ms
        self.dwell_time = dwell_time
        self.left_offset = left_offset

        self.center_pulse = 1500
        self.left_pulse = 1250
        self.next_switch_time = 0.0
        self.go_left_next = True

    def configure_from_center(self, center_pulse):
        self.center_pulse = int(center_pulse)
        self.left_pulse = int(center_pulse + self.left_offset)
        self.next_switch_time = 0.0
        self.go_left_next = True

    def center(self):
        Board.setPWMServoPulse(self.servo_id, self.center_pulse, self.move_time_ms)

    def update(self, now):
        if now < self.next_switch_time:
            return

        if self.go_left_next:
            Board.setPWMServoPulse(self.servo_id, self.left_pulse, self.move_time_ms)
        else:
            Board.setPWMServoPulse(self.servo_id, self.center_pulse, self.move_time_ms)

        self.go_left_next = not self.go_left_next
        self.next_switch_time = now + self.dwell_time


# ----------------------------
# Camera calibration
# ----------------------------
param_data = np.load(calibration_param_path + '.npz')
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)






def main():
    # 1. Register with GameController and get ID
    conn = RobotConnectionHandler(role="guard")
    guard_id, broker_ip = conn.register_and_wait_for_id()

    print(f"[GUARD] Assigned ID = {guard_id}, broker = {broker_ip}")

    # 2. Create the runtime MQTT client
    comm = RobotComm(
        client_name=guard_id,
        broker_ip=broker_ip,
        subscriptions=[
            f"game/guard/{guard_id}/command",
            "game/system/#"
        ],
        on_message=on_message,   
        heartbeat_interval=5.0,
        heartbeat_prefix="game/robot"
    )

    comm.connect()





    patrol = VisualPatrolController()
    follow = FollowController(required_detect_seconds=15.0, use_hsv=True)
    pan = LeftCenterPan()

    patrol.init()
    follow.init()

    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()

    def enter_patrol():
        try:
            follow.stop()
        except Exception:
            pass

        try:
            patrol.stop()
        except Exception:
            pass

        patrol.start()
        patrol.set_line_target_color(('black',))

        try:
            AGC.runActionGroup('stand')
        except Exception:
            pass

        try:
            patrol.look_down_at_floor()
        except Exception:
            pass

        time.sleep(0.2)
        return STATE_PATROL, time.monotonic(), None

    def enter_search():
        try:
            patrol.stop()
        except Exception:
            pass

        try:
            follow.stop()
        except Exception:
            pass

        follow.start()
        follow.set_ball_target_color(('red',))

        # Force the head into the search pose from guard_main
        # servo 1 = pitch (up/down)
        # servo 2 = yaw (left/right)
        center_yaw = follow.get_center_yaw()

        Board.setPWMServoPulse(1, LOOK_UP_PULSE, HEAD_MOVE_MS)
        Board.setPWMServoPulse(2, center_yaw, HEAD_MOVE_MS)
        time.sleep(HEAD_MOVE_MS / 1000.0)

        pan.configure_from_center(center_yaw)
        pan.center()

        return STATE_SEARCH, time.monotonic(), None

    state, state_start_time, last_target_seen_time = enter_patrol()

    try:
        while True:
            ret, img = my_camera.read()
            if not ret or img is None:
                time.sleep(0.01)
                continue

            frame = cv2.remap(img.copy(), mapx, mapy, cv2.INTER_LINEAR)
            now = time.monotonic()

            if state == STATE_PATROL:
                send_event("patrol_started")
                display = patrol.run(frame)

                if now - state_start_time >= PATROL_SECONDS:
                    state, state_start_time, last_target_seen_time = enter_search()

            elif state == STATE_SEARCH:
                send_event("search_started")
                display = follow.run(frame)

                if follow.target_caught == True:
                    game_over = True

                if follow.target_visible():
                    last_target_seen_time = now
                    send_detection("target_visible")
                else:
                    pan.update(now)
                

                if last_target_seen_time is None and (now - state_start_time >= SEARCH_SECONDS):
                    state, state_start_time, last_target_seen_time = enter_patrol()

                elif last_target_seen_time is not None and (now - last_target_seen_time >= LOST_TARGET_TIMEOUT):
                    state, state_start_time, last_target_seen_time = enter_patrol()

            else:
                display = frame

            cv2.imshow('guard_main', display)
            key = cv2.waitKey(1)
            if key == 27:
                break

            time.sleep(0.01)

    finally:
        try:
            patrol.stop()
        except Exception:
            pass

        try:
            follow.stop()
        except Exception:
            pass

        try:
            patrol.look_down_at_floor()
        except Exception:
            pass

        try:
            if open_once:
                my_camera.release()
            else:
                my_camera.camera_close()
        except Exception:
            pass

        cv2.destroyAllWindows()


    # ----------------------
    # MQTT Helper Functions
    # ----------------------

        # on message for mqtt
    def on_message(client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == f"game/guard/{guard_id}/command":
            print(f"[GUARD] Received command: {payload}")
            # You can hook this into your FSM if needed

    def send_event(event):
        comm.publish(f"game/guard/{guard_id}/events", event)

    def send_detection(det):
        comm.publish(f"game/guard/{guard_id}/detections", det)


if __name__ == '__main__':
    main()