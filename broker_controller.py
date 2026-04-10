# broker_controller.py
'''
This file contains the class responsible for acting as a broker to the robots. 
The intention is to use these functions in combination with a GUI.
'''

from robot_functions.robot_comm import RobotComm
import json
import time

class BrokerController:
    def __init__(self, client_name, broker_ip, on_robot_seen=None, on_heartbeat=None):
        self.on_robot_seen = on_robot_seen 
        self.on_heartbeat = on_heartbeat

        self.comm = RobotComm(
            client_name=client_name,
            broker_ip=broker_ip,
            subscriptions=["robot/+/heartbeat"],      # broker mostly publishes; can add subs later
            on_message=self._on_message,
            heartbeat_interval=5.0, # optional, or disable if you want
            heartbeat_prefix="controller"
        )
        self.comm.connect()

    # ---------- ROLE ASSIGNMENT ---------- #

    def assign_role(self, robot_name, role, role_id=None):
        """
        role: "guard", "intruder", "idle", etc.
        role_id: optional numeric ID for guard/intruder
        """
        topic = f"game/role/{robot_name}"
        payload = {"role": role}
        if role_id is not None:
            payload["id"] = role_id

        self.comm.publish(topic, json.dumps(payload))
        print(f"[BROKER] Assigned role {payload} to {robot_name}")

    # ---------- ACTION GROUPS ---------- #

    def run_action_group(self, robot_names, group_name):
        """
        robot_names: LIST of robot name strings -- for one robot use list(robot_name) or [robot_name]
        group_name: name of the action group to run
        """
        for name in robot_names:
            topic = f"robot/{name}/action_group"
            self.comm.publish(topic, group_name)
            print(f"[BROKER] {name} → action_group: {group_name}")

    # ---------- SCRIPTS ---------- #

    def run_script(self, robot_names, script_name):
        """
        robot_names: LIST of robot name strings -- for one robot use list(robot_name) or [robot_name]
        script_name: name of the script to run
        """
        for name in robot_names:
            topic = f"robot/{name}/script"
            self.comm.publish(topic, script_name)
            print(f"[BROKER] {name} → script: {script_name}")

    # ---------- STOP COMMAND ---------- #

    def stop_robots(self, robot_names):
        """
        robot_names: LIST of robot name strings -- for one robot use list(robot_name) or [robot_name]
        """
        for name in robot_names:
            topic = f"robot/{name}/stop"
            self.comm.publish(topic, "STOP")
            print(f"[BROKER] {name} → STOP")

    # ------ subscription handler ------ #
    def _on_message(self, client, userdata, msg):
        print("[BROKER] Received:", msg.topic, msg.payload)
        topic = msg.topic

        if topic.startswith("robot/") and topic.endswith('/heartbeat'):
            robot_name = topic.split("/")[1]

            if self.on_robot_seen:
                self.on_robot_seen(robot_name)

            if self.on_heartbeat:
                self.on_heartbeat(robot_name)



    # ---------- OPTIONAL: BROADCAST HELPERS ---------- #

    def broadcast_action_group(self, group_name, all_robot_names):
        self.run_action_group(all_robot_names, group_name)

    def broadcast_stop(self, all_robot_names):
        self.stop_robots(all_robot_names)
