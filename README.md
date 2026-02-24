# TonyPi Example Scripts

This repository contains a self‑contained example environment for running the
`Follow.py` and `VisualPatrol.py` scripts originally distributed with the
HiWonder TonyPi robot.  It bundles simplified versions of the HiWonder
Python SDK, colour and servo configuration files, example action group
files, and a dummy camera calibration file.  These resources allow you
to experiment with the code on a desktop computer without a robot
connected.

## Directory structure

```
tonypi_project/
├── Follow.py                     # Object tracking script
├── VisualPatrol.py               # Line‑following script
├── README.md                     # This documentation
├── requirements.txt              # Python dependencies
├── servo_config.yaml             # Servo initial positions
├── lab_config.yaml               # LAB colour thresholds
├── CameraCalibration/
│   ├── CalibrationConfig.py      # Calibration paths
│   └── calibration_param.npz     # Dummy camera calibration data
├── ActionGroups/
│   ├── stand_slow.d6a            # Action group files
│   ├── go_forward.d6a
│   ├── turn_left_small_step.d6a
│   ├── turn_right_small_step.d6a
│   └── back_fast.d6a
└── hiwonder/
    ├── __init__.py
    ├── PID.py                    # PID controller
    ├── Camera.py                 # Camera wrapper
    ├── Misc.py                   # Helper functions
    ├── Controller.py             # Servo/motor controller stubs
    ├── ActionGroupControl.py     # Action group player stub
    ├── common.py                 # Colour picker helper
    ├── yaml_handle.py            # Config loader
    └── ros_robot_controller_sdk.py # Board stub
```

## Installation

1. Clone or download this repository and change into its directory:

```bash
git clone <your‑repo‑url>
cd tonypi_project
```

2. Install the Python dependencies:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

The dependencies include `opencv-python`, `numpy`, `PyYAML` and `pandas`.

## Running the scripts

Both scripts require access to a webcam.  On most systems the default
camera is index 0.  The scripts will open the camera, perform colour
detection and send simulated servo/motor commands to the console.

### Object tracking (`Follow.py`)

To run the object tracking script, execute:

```bash
python3 Follow.py
```

Select the colour to track by calling `setBallTargetColor('red')`,
`'green'`, `'blue'` etc.  In this example environment the servo and
motor commands are logged via the Python `logging` module.

### Line following (`VisualPatrol.py`)

To run the line‑following script:

```bash
python3 VisualPatrol.py
```

Click on the displayed image to choose a colour to follow.  The script
will look for contours of the selected colour in predefined regions of
the frame and simulate robot movement by running action groups such as
`go_forward` and `turn_left_small_step`.

## Notes

* The HiWonder SDK has been replaced by stubs; therefore servo and
  motor commands do not control real hardware.  You can use this
  repository as a starting point and replace the stubs with the
  genuine SDK when running on a real TonyPi robot.
* The LAB colour thresholds provided in `lab_config.yaml` are
  illustrative.  For reliable detection you should adjust these
  thresholds using the LAB tool supplied with the HiWonder robot.
* The calibration parameters in `calibration_param.npz` are dummy
  values.  If you have performed camera calibration, replace this
  file with your own parameters (arrays named `mtx_array` and
  `dist_array`).
"# TonyPi-StealthGame" 
