# Create fake calibration file
import numpy as np


np.savez("robot_functions/guard/stubs/CameraCalibration/fake_calibration.npz",
         mtx_array=np.eye(3),
         dist_array=np.zeros(5))

print("Created fake_calibration.npz in current directory")
