# main_robot.py
import time
import yaml
import sys
import os 

from robot_comm import RobotComm
from robot_role import RobotRole
from robot_controller_fake import TonyPiController  # your movement API

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from hiwonder.yaml_handle import load_robot_config

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
