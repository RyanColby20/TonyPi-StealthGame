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

        self.robot_registry = {}  
        # structure:
        # {
        #   "<mac>": {"role": "guard", "id": "guard1", "last_seen": timestamp}
        # }

        self.game_running = False

        self.comm = RobotComm(
            client_name="game_controller",
            broker_ip=broker_ip,
            subscriptions=[
                "game/robot/+/heartbeat", # changed heartbeat to game/robot
                "game/guard/+/events",
                "game/guard/+/detections",
                "game/intruder/+/events",
                "game/robot/register"
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


        # -----------------------------------------
        # ID ASSIGN
        # -----------------------------------------
        if topic == "game/robot/register":
            data = json.loads(payload)
            role = data.get("role")
            mac = data.get("mac")

            if not role or not mac:
                print("[GAME] Invalid registration payload:", payload)
                return

            assigned_id = self._assign_id(role)

            # Save robot in registry
            self.robot_registry[mac] = {
                "role": role,
                "id": assigned_id,
                "last_seen": time.time()
            }

            print(f"[GAME] Registered {role} with MAC={mac} → assigned ID={assigned_id}")

            # Send assignment back to robot
            self.comm.publish(
                f"game/robot/role_assignment/{mac}",
                json.dumps({"id": assigned_id})
            )
            return

        # -----------------------------------------
        # HEARTBEATS
        # -----------------------------------------
        if topic.startswith("game/robot/") and topic.endswith("/heartbeat"):
            client_name = topic.split("/")[2]
            now = time.time()

            for mac, info in self.robot_registry.items():
                if info["id"] in client_name:   # e.g. "guard1" in "guard_guard1"
                    info["last_seen"] = now
                    break

            return

        # -----------------------------------------
        # GUARD EVENTS
        # -----------------------------------------
        if topic.startswith("game/guard/") and topic.endswith("/events"):
            guard_id = topic.split("/")[2]
            event = payload

            if self.on_event:
                self.on_event({"type": "guard_event", "guard": guard_id, "event": event})

            # Example: guard reports "intruder_captured"
            if event == "intruder_captured":
                self.trigger_game_over(winner="guard", guard_id=guard_id)

            return

        # -----------------------------------------
        # GUARD DETECTIONS
        # -----------------------------------------
        if topic.startswith("game/guard/") and topic.endswith("/detections"):
            guard_id = topic.split("/")[2]
            detection = payload

            if self.on_event:
                self.on_event({"type": "guard_detection", "guard": guard_id, "detection": detection})

            return

        # -----------------------------------------
        # INTRUDER EVENTS
        # -----------------------------------------
        if topic.startswith("game/intruder/") and topic.endswith("/events"):
            intruder_id = topic.split("/")[2]
            event = payload

            if self.on_event:
                self.on_event({"type": "intruder_event", "intruder": intruder_id, "event": event})

            # Example: intruder reports "win"
            if event == "WIN":
                self.trigger_game_over(winner="intruder", intruder_id=intruder_id)

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
        self.comm.publish("game/system/reset", "RESET")
        self._notify_event("Game reset")

    def _handle_detection(self, guard_id, payload):
        # Example: end game on detection
        self._notify_event(f"Game Over: Guard {guard_id} caught intruder")
        self.comm.publish("game/system/game_over", f"guard {guard_id}")
        self.stop_game()

    # ---------------------------------------------------------
    # ROBOT-SPECIFIC COMMANDS
    # ---------------------------------------------------------
    def send_guard_command(self, guard_id, command):
        topic = f"game/guard/{guard_id}/command"
        self.comm.publish(topic, command)

    def send_intruder_command(self, intruder_id, command):
        topic = f"game/intruder/{intruder_id}/command"
        self.comm.publish(topic, command)

    # ---------------------------------------------------------
    # ID ASSIGNMENT HELPER
    # ---------------------------------------------------------

    def _assign_id(self, role: str) -> str:
        """
        Assigns the next available ID for a given role.
        Example:
            guard → guard1, guard2, guard3...
            intruder → intruder1, intruder2...
        """
        # Count existing robots with this role
        count = sum(1 for r in self.robot_registry.values() if r["role"] == role)
        return f"{role}{count + 1}"


    # ---------------------------------------------------------
    # GUI CALLBACK HELPERS
    # ---------------------------------------------------------
    def _notify_event(self, msg):
        if self.on_event:
            self.on_event(msg)
