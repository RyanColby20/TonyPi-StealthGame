'''
 robot_connection.py
 this file handles client-side connections 
 specifically for the intruder-guard game

'''

# robot_connection.py

import json
import time
import uuid

from HiwonderSDK.yaml_handle import load_robot_config
from robot_functions.robot_comm import RobotComm


class RobotConnectionHandler:
    def __init__(self, role: str):
        """
        role: "guard" or "intruder"
        """
        self.role = role
        self.mac = hex(uuid.getnode())

        # Load broker IP from YAML
        config = load_robot_config()
        self.broker_ip = config.get("robot", {}).get("broker_ip", "127.0.0.1")

        self.assigned_id = None

        # Minimal temporary comm just for registration/assignment
        self.comm = RobotComm(
            client_name=f"bootstrap_{self.role}_{self.mac}",
            broker_ip=self.broker_ip,
            subscriptions=[f"game/robot/role_assignment/{self.mac}"],
            on_message=self._on_message,
            heartbeat_interval=999999.0,      # no heartbeat needed here
            heartbeat_prefix=f"bootstrap/heartbeat"
        )

        self.comm.connect()

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == f"game/robot/role_assignment/{self.mac}":
            data = json.loads(payload)
            self.assigned_id = data["id"]
            print(f"[BOOTSTRAP] Assigned id={self.assigned_id} for role={self.role}")

    def register_and_wait_for_id(self, timeout=10.0):
        """
        Send registration request and block until ID is assigned or timeout.
        """
        payload = json.dumps({"role": self.role, "mac": self.mac})
        self.comm.publish("game/robot/register", payload)

        start = time.time()
        while self.assigned_id is None and (time.time() - start) < timeout:
            time.sleep(0.05)

        if self.assigned_id is None:
            raise TimeoutError("Did not receive role_assignment from GameController")

        # Done with bootstrap comm; caller will create its own RobotComm
        self.comm.disconnect()
        return self.assigned_id, self.broker_ip
