"""Camera wrapper for OpenCV video capture.

This module provides a simple `Camera` class that wraps OpenCV's
`VideoCapture` interface and a pair of convenience functions to
open and close the default camera.  The original HiWonder SDK
includes additional functionality; this implementation focuses on
the API required by Follow.py and VisualPatrol.py.
"""

import cv2


class Camera:
    """A simple wrapper around cv2.VideoCapture."""

    def __init__(self, index: int = 0, resolution=(640, 480)):
        self.index = index
        self.resolution = resolution
        self.cap = None

    def camera_open(self):
        """Open the camera device."""
        if self.cap is not None:
            return True
        self.cap = cv2.VideoCapture(self.index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        return self.cap.isOpened()

    def camera_close(self):
        """Release the camera device."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def frame(self):
        """Capture a single frame.

        Returns:
            frame (ndarray): The captured image in BGR format.
            None: if the capture failed.
        """
        if self.cap is None:
            if not self.camera_open():
                return None
        ret, frame = self.cap.read()
        return frame if ret else None


# Convenience functions emulating the original HiWonder API
_global_camera = Camera()


def camera_open(resolution=(640, 480)):
    """Open the global camera with the given resolution."""
    _global_camera.resolution = resolution
    return _global_camera.camera_open()


def camera_close():
    """Close the global camera."""
    _global_camera.camera_close()


def get_frame():
    """Get a single frame from the global camera."""
    return _global_camera.frame()