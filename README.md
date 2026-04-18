# TonyPi Stealth Game and Network Framework

This project is composed of two primary elements built on top of the default TonyPi framework. The TonyPi's are connected to a singular wifi network. This project then allow for synchronous Action Group or Script execution. The second layer of this project is an intruders vs guards stealth game. The game is managed by a GUI and each guard is an autonomous patrol based robot to be avoided by a player's remote-controlled intruder.


Todo

## Directory structure

```
Todo at end: tree
```

## Installation
### Robot instructions
1. Download the contents of this repo into the /TonyPi/ folder. 
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

TODO

## Notes


"# TonyPi-StealthGame" 
