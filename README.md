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
|   ├── stand.d6a
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

## How the Codes Work and What They Do:

### VisualPatrol.py

The visual patrol file is customized to control the patrol behavior of the TonyPi Pro. What it does in a general sense is that it makes the robot follow a floor line, that is in default black, while also pausing and scanning for an intruder. The intruder however will be marked with a red signifier. So in theory, it is really just looking for the color red, which is the intruder in our case. If the red is detected, then the control is directed to the follow.py code file. However here is a more formal definition of the file:

* The robot follows the line using the camera by checking a few “zones” near the bottom of the image and estimating where the line is, then it adjusts its movement to stay centered.
* Every few seconds, it pauses patrol, looks up, and does a slow left-to-right head scan while keeping the camera feed updating.
* If it sees enough red during the scan, it stops patrol and launches Follow.py, saving the head position so following starts from the same direction it detected the intruder.

This is a generalization of this file, and is a core component of this project.

### Follow.py

The follow file in this project is the file that is also used in our visual patrol file. As mentioned before, the visual patrol file runs then when an intruder is detected, the follow file is then activated. The follow file serves the purpose to use the camera that the tonypi pro is equipped with to continuosly look for the target color, which in our case for the intruder is red. While it is looking for that color, it will continuously adjust its head and movement to keep the target color centered. Now if the target moves, it will turn and either step forward or backward to follow it. But, if the target is continuosly detected, for the set amount of time that we desire, the robot will then finally trigger a win action, from the action group. Here is a more formal defintion of the file:

* Detects the target color (red) using the camera and LAB color thresholds.
* Finds the largest red object and calculates its center position in the frame.
* Uses PID control to adjust the robot’s head servos so the target stays centered. (PID control makes the robot do smooth movements instead of jerky, unstable movements)
* Moves the robot (turn left/right, forward, or backward) to follow the target.
* If the target is detected continuously for set time, the robot performs a “win” action.

This is a generalization of this file, and is a core component of this project.

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
