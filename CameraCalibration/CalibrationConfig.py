"""Camera calibration configuration.

This module defines the paths used to store and load camera
calibration parameters.  The original HiWonder SDK stores these
files in a system directory.  Here we place them inside the
project's `CameraCalibration` folder so that they are included
with the repository.  Scripts using these parameters should
import `calibration_param_path` and append `.npz` to read or
write the calibration data.
"""

import os

# Determine the directory containing this file
_dir = os.path.dirname(__file__)

# The folder into which calibration files will be saved
calibration_path = _dir

# The base path (without extension) of the calibration parameter file
calibration_param_path = os.path.join(calibration_path, 'calibration_param')