# TonyPi Stealth Game and Network Framework

This project is composed of two primary elements built on top of the default TonyPi framework. The TonyPi's are connected to a singular wifi network. This project then allow for synchronous Action Group or Script execution. The second layer of this project is an intruders vs guards stealth game. The game is managed by a GUI and each guard is an autonomous patrol based robot to be avoided by a player's remote-controlled intruder. This project also includes a GUI to demonstrate some of the network functionalities of the robots at the Brandywine Student Expo.

## Directory structure

```
Folder PATH listing for volume Windows
Volume serial number is CADF-5C02
C:.
|   broker_controller.py
|   EXPO_GUI_RUN_ME.py
|   git-friend.py
|   gui_game_controller.py
|   gui_robot_broker.py
|   lab_config.yaml
|   lab_config_hsv.yaml
|   main_robot.py
|   README.md
|   requirements.txt
|   robot_config.yaml
|   servo_config.yaml
|   tree.txt
|   
+---ActionGroups
|       
+---game_functions
|       GameController.py
|       
+---robot_functions
|   |   robot_comm.py
|   |   robot_controller.py
|   |   robot_controller_dummy.py
|   |   robot_role.py
|   |   __init__.py
|   |   
|   +---game
|   |       robot_connection.py
|   |       
|   +---guard
|   |   |   follow_class.py
|   |   |   guard_main.py
|   |   |   visualPatrol_class.py
|   |   |   __init__.py
|   |   |   
|   |   \---stubs
|   |       |   __init__.py
|   |       |   
|   |       +---CameraCalibration
|   |       |       CalibrationConfig.py
|   |       |       fake_calibration.npz
|   |       |       npz_stub_gen.py
|   |       |       __init__.py
|   |       |       
|   |       \---HiwonderSDK
|   |               ActionGroupControl.py
|   |               Board.py
|   |               Camera.py
|   |               Misc.py
|   |               PID.py
|   |               __init__.py
|   |               
|   \---intruder
|           intruder_main.py
|           Joystick.py
|           __init__.py
|           
+---utils
|       yaml_handle.py


```

## Installation
### Robot instructions
1. Download the contents of this repo into the /TonyPi/ folder (alongside the hiwonder library). 
2. Install the Python dependencies:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

The dependencies include `opencv-python`, `numpy`, `PyYAML`, `paho-mqtt` and `pandas`.

### Broker instructions
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

The dependencies include `opencv-python`, `numpy`, `PyYAML`, `paho-mqtt` and `pandas`.

3. Install a MQTT broker service. For this project we used Eclipse Mosquitto. https://mosquitto.org

## Setting up the robots.

To run any component of this project, each robot must be configured to connect to a singular WiFi Network. Many modern windows computers are capable of hosting a WiFi hotspot with internet sharing from their internet source. This is the simplest method of hosting. To configure each robot to connect to the network: 

1. Find hw_wifi.py in /hiwonder-toolbox/
2. in main() find the following variables: 
```
HW_WIFI_MODE = 2    # Change from 1 (AP mode) to 2 (STA mode)
HW_WIFI_STA_SSID = 'YourNetwork'    # Your desired WiFi network's name
HW_WIFI_STA_PASSWORD = 'YourNetworkPassword'  # Your desired WiFi network's password
```
3. Additionally, wpa_supplicant.conf may be referenced by hw_wifi.py. This file may not exist. If it does exist: modify or remove as needed.
4. Save and restart the WiFi service
```
sudo systemctl restart hw_wifi.service
```
5. Connect to the Pi using SSH, use a program such as RealVNC Viewer if desired.

## Running the project

There are three runnable components of this project. 
For all three components on each robot being used and synchronised, simply run main_robot.py using python once the robot is connect to the appropriate network.

### Expo GUI
This runnable component is the GUI we used to demonstrate the project at the Brandywine Student Expo. This component is a simple GUI that allows selection of one to all of the connected robots to synchronously run preselected action groups. To run this component, using the MQTT broker, simply run the file EXPO_GUI_RUN_ME.py using python.

### Robot Broker GUI
The second runnable component of this project is the Robot Broker GUI. This component allows the user to select any to all robots connected then issue commands to run any ActionGroup or Script present in the files. To run this component, using the MQTT broker, simply run the file gui_robot_broker.py using python.

### Game Controller GUI
This component of the project is used to manage the Guards vs Intruders Stealthgame. This component requires it's GUI must run before any guard or intruder scripts are executed. To begin, using the MQTT broker simply run the file gui_game_controller.py using python. Then, using the robot broker GUI, select and execute guard script and intruder script on the appropriate robots. From there, using the start game button begin the game.

## How the Codes Work and What They Do:

### `main_robot.py`
Main file that runs on each TonyPi robot. It connects the robot to MQTT and listens for commands to run action groups, scripts, or stop.

### `broker_controller.py`
Backend controller for the robot broker GUI. It sends MQTT commands to selected robots and tracks robot heartbeats.

### `gui_robot_broker.py`
General robot-control GUI. It lets the user select robots and manually send action groups or scripts.

### `EXPO_GUI_RUN_ME.py`
Simplified demo GUI for the Student Expo. It provides preset buttons for common robot actions like wave, bow, kick, dance, and stop, and puts them on display.

### `game_functions/GameController.py`
Main controller for the Guards vs Intruder game. It registers robots, assigns IDs, sends start/stop/reset commands, tracks heartbeats, and handles game-over events. Pretty much the heart of the system.

### `gui_game_controller.py`
GUI for managing the stealth game. It lets the user start, stop, reset, and monitor the game.

### `robot_functions/robot_comm.py`
MQTT communication wrapper used by the robots. It handles connecting, subscribing, publishing messages, and sending heartbeat signals as well.

### `robot_functions/robot_controller.py`
Real TonyPi controller used to run robot action groups and scripts. It connects high-level commands like `guard` or `intruder` to the actual robot behavior code.

### `robot_functions/robot_controller_dummy.py`
Testing version of the TonyPi controller. It allows the code to be tested on a regular computer without physical robot hardware.

### `robot_functions/robot_role.py`
Tracks the robot’s current role or state. It helps prevent the robot from running multiple actions or scripts at the same time, which can cause concurrency issues.

### `robot_functions/game/robot_connection.py`
Handles guard and intruder registration with the game controller. It sends the robot’s role and MAC address, then receives the assigned game ID.

### `robot_functions/guard/guard_main.py`
Main behavior script for a guard robot. It waits for the game to start, patrols, searches for the intruder, and reports when the intruder is caught.

### `robot_functions/guard/visualPatrol_class.py`
Controls the guard’s patrol behavior. It uses the camera to detect and follow a black line on the floor.

### `robot_functions/guard/follow_class.py`
Controls the guard’s search/follow behavior. It detects the red intruder target and moves the guard toward it.

### `robot_functions/intruder/intruder_main.py`
Main behavior script for the intruder robot. It waits for the game to start, then enables joystick-controlled movement.

### `robot_functions/intruder/Joystick.py`
Reads joystick input using `pygame`. It converts controller movements and button presses into TonyPi action groups.

### `utils/yaml_handle.py`
Utility file for reading and writing YAML configuration files. It helps load robot, servo, and color-detection settings.

### `lab_config.yaml`
Stores LAB color threshold settings used for vision detection. These values help the robot identify colors like the black patrol line.

### `lab_config_hsv.yaml`
Stores HSV color threshold settings used for vision detection. These values help with detecting colored targets such as the red intruder marker.

### `robot_config.yaml`
Stores robot-specific settings such as the robot name, broker IP address, and MQTT topic configuration.

### `servo_config.yaml`
Stores servo-related configuration values for the TonyPi robot. It helps define how the servos are calibrated or positioned.

### `requirements.txt`
Lists the Python packages needed to run the project. This includes libraries such as OpenCV, NumPy, PyYAML, Paho MQTT, and Pygame.
