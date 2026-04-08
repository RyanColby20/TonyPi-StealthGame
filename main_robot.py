# main_robot.py
'''
This is the main file to run on each robot.
The robot will initialize into an idle and listen mode, awaiting commands.
Note that behavior and properties used by this file are in robot_config.yaml.
This includes an initial script to execute on startup of this (main_robot) file.

Note if you are having connection rejected issues with MQTT:
the broker IP should almost always be 192.168.137.1
if not, check the ip on the broker with ipconfig.
'''

import time
import yaml
import sys
import os 

from robot_functions.robot_comm import RobotComm
from robot_functions.robot_role import RobotRole
from robot_functions.robot_controller_dummy import TonyPiController  # your movement API
from HiwonderSDK.yaml_handle import load_robot_config

from pathlib import Path

sdk_path = Path(__file__).resolve().parent.parent / "HiWonderSDK" / "hiwonder"
sys.path.insert(0, str(sdk_path))

# --------- MQTT on message ------------ #

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()

    print(f"[MQTT] {topic} → {payload}")

    if topic.endswith("/action_group"):
        role.handle_action_group(payload)

    elif topic.endswith("/script"):
        role.handle_script(payload)

    elif topic.endswith("/stop"):
        role.stop_current()

# ---------------- MAIN ---------------- #

if __name__ == "__main__":
    # config load
    config = load_robot_config()

    client_name = config["robot"]["client_name"]
    broker_ip = config["robot"]["broker_ip"]
    subscriptions = config["mqtt"]["subscriptions"]

    robot = TonyPiController()
    comm = RobotComm(
        client_name=client_name,
        broker_ip=broker_ip,
        subscriptions=subscriptions,
        on_message=on_message
    )

    role = RobotRole(comm, robot)

    comm.connect()

    print("[MAIN] Robot entering idle mode")

    # Optional: run initial script
    initial_script = config.get("startup", {}).get("initial_script")
    if initial_script:
        print(f"[MAIN] Running initial script: {initial_script}")
        role.handle_script(initial_script)


    while True:
        time.sleep(1)
