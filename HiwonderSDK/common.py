"""Common utilities for colour selection and detection.

This module provides a simplified `ColorPicker` class used by
VisualPatrol.py.  The original HiWonder implementation presents a
GUI with trackbars allowing the user to adjust thresholds.  Here
we implement a basic colour sampler: when instantiated with a
position and radius, calling the object with an image returns
the LAB and RGB colour at that point along with a copy of the
image annotated with a marker.
"""

import cv2
import numpy as np


class ColorPicker:
    """Pick the colour at a specific point in an image.

    Args:
        pos (tuple): (x, y) coordinates of the point in the image.
        radius (int): Radius of the sampling region.
    """

    def __init__(self, pos, radius=10):
        self.x, self.y = pos
        self.radius = radius
        self.color_lab = None
        self.color_rgb = None
        self.picked = False

    def __call__(self, frame, display_frame):
        """Sample the colour at the stored position.

        Args:
            frame (ndarray): The source BGR image.
            display_frame (ndarray): A copy of the frame on which
                markers can be drawn.

        Returns:
            tuple: (lab_color, rgb_color) if a colour has been
                picked; otherwise an empty list.
            ndarray: The annotated display frame.
        """
        h, w = frame.shape[:2]
        x0 = max(0, self.x - self.radius)
        x1 = min(w, self.x + self.radius)
        y0 = max(0, self.y - self.radius)
        y1 = min(h, self.y + self.radius)

        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            return [], display_frame

        # Compute mean colour in LAB and RGB spaces
        roi_lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        mean_lab = roi_lab.reshape(-1, 3).mean(axis=0)
        mean_bgr = roi.reshape(-1, 3).mean(axis=0)
        mean_rgb = mean_bgr[::-1]  # Convert BGR to RGB order

        # Save the picked colour
        self.color_lab = mean_lab
        self.color_rgb = mean_rgb
        self.picked = True

        # Draw a circle on the display frame to show the sampled region
        cv2.circle(display_frame, (self.x, self.y), self.radius, (0, 255, 0), 2)

        # Return colours as lists for YAML compatibility
        return ([int(v) for v in self.color_lab], [int(v) for v in self.color_rgb]), display_frame