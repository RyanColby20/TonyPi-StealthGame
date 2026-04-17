# HiwonderSDK/Camera.py

class Camera:
    def __init__(self):
        print("[FAKE Camera] Camera() created")

    def camera_open(self):
        print("[FAKE Camera] camera_open() called")

    def read(self):
        # print("[FAKE Camera] read() called")
        # Return fake frame: (success, None)
        return True, None

    def camera_close(self):
        print("[FAKE Camera] camera_close() called")
