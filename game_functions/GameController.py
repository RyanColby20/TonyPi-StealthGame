# game_controller.py

'''
This file acts as the Game Master of the Intruders vs Guards game. 

'''

import json
import time
from robot_functions.robot_comm import RobotComm

class GameController:
    def __init__(self, broker_ip, on_event=None, on_role_update=None):
        self.on_event = on_event
        self.on_role_update = on_role_update

        self.guards = {}       # guard_id → robot_name
        self.intruders = {}    # intruder_id → robot_name
        self.robot_roles = {}  # robot_name → role

        self.game_running = False

        self.comm = RobotComm(
            client_name="game_controller",
            broker_ip=broker_ip,
            subscriptions=[
                "game/robot/+/heartbeat", # changed heartbeat to game/robot
                "robot/+/stop", # this may cause socket issues ?
                "game/role/+",
                "game/guard/+/events",
                "game/guard/+/detections",
                "game/intruder/+/events"
            ],
            on_message=self._on_message,
            heartbeat_interval=5.0,
            heartbeat_prefix="system"
        )

        self.comm.connect()

    # ---------------------------------------------------------
    # MQTT HANDLER
    # ---------------------------------------------------------
    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        # Robot heartbeat
        if topic.startswith("game/robot/") and topic.endswith("/heartbeat"):
            robot_name = topic.split("/")[1]
            self._notify_event(f"Heartbeat from {robot_name}")
            return

        # Robot STOP → stop entire game
        if topic.startswith("robot/") and topic.endswith("/stop"):
            robot_name = topic.split("/")[1]
            self._notify_event(f"STOP received from {robot_name}, halting game")
            self.stop_game()
            return

        # Role assignment
        if topic.startswith("game/role/"):
            robot_name = topic.split("/")[2]
            data = json.loads(payload)
            role = data.get("role")
            role_id = data.get("id")

            self.robot_roles[robot_name] = role

            if role == "guard":
                self.guards[role_id] = robot_name
            elif role == "intruder":
                self.intruders[role_id] = robot_name

            if self.on_role_update:
                self.on_role_update(robot_name, role, role_id)

            self._notify_event(f"{robot_name} assigned role {role} ({role_id})")
            return

        # Guard detections
        if "game/guard" in topic and "/detections" in topic:
            guard_id = topic.split("/")[2]
            self._notify_event(f"Guard {guard_id} detected intruder: {payload}")
            self._handle_detection(guard_id, payload)
            return

        # Guard events
        if "game/guard" in topic and "/events" in topic:
            guard_id = topic.split("/")[2]
            self._notify_event(f"Guard {guard_id} event: {payload}")
            return

        # Intruder events
        if "game/intruder" in topic and "/events" in topic:
            intruder_id = topic.split("/")[2]
            self._notify_event(f"Intruder {intruder_id} event: {payload}")
            return

    # ---------------------------------------------------------
    # GAME LOGIC
    # ---------------------------------------------------------
    def start_game(self):
        self.game_running = True
        self.comm.publish("game/system/start", "START")
        self._notify_event("Game started")

    def stop_game(self):
        self.game_running = False
        self.comm.publish("game/system/stop", "STOP")
        self._notify_event("Game stopped")

    def reset_game(self):
        self.game_running = False
        self.guards.clear()
        self.intruders.clear()
        self.robot_roles.clear()
        self.comm.publish("game/system/reset", "RESET")
        self._notify_event("Game reset")

    def _handle_detection(self, guard_id, payload):
        # Example: end game on detection
        self._notify_event(f"Game Over: Guard {guard_id} caught intruder")
        self.comm.publish("game/system/game_over", f"guard {guard_id}")
        self.stop_game()

    # ---------------------------------------------------------
    # GUI CALLBACK HELPERS
    # ---------------------------------------------------------
    def _notify_event(self, msg):
        if self.on_event:
            self.on_event(msg)
