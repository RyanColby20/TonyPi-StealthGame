# intruder_main.py
'''
This file runs the state machine responsible for managing the intruder
This is the file that should be run to initialize an intruder within the game.
'''

import json
import time
from robot_functions.robot_comm import RobotComm

class intruder_main:
    def __init__(self, intruder_id, broker_ip, robot_controller):
        self.id = intruder_id
        self.robot = robot_controller

        self.state = "WAITING_FOR_GAME_START"

        self.comm = RobotComm(
            client_name=f"intruder_{intruder_id}",
            broker_ip=broker_ip,
            subscriptions=[
                "game/system/start",
                "game/system/stop",
                "game/system/reset",
                "game/system/game_over"
            ],
            on_message=self._on_message,
            heartbeat_interval=5.0,
            heartbeat_prefix="intruder"
        )

        self.comm.connect()

    # ---------------------------------------------------------
    # MQTT HANDLER
    # ---------------------------------------------------------
    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == "game/system/start":
            self._start_game()

        elif topic == "game/system/stop":
            self._stop_game()

        elif topic == "game/system/reset":
            self._reset_game()

        elif topic == "game/system/game_over":
            self._handle_game_over(payload)

    # ---------------------------------------------------------
    # STATE TRANSITIONS
    # ---------------------------------------------------------
    def _start_game(self):
        if self.state == "WAITING_FOR_GAME_START":
            self.state = "ACTIVE_PLAYER_CONTROL"
            self._publish_state()
            print("[INTRUDER] Game started, player control enabled")

    def _stop_game(self):
        self.state = "WAITING_FOR_GAME_START"
        self.robot.stop_all()
        self._publish_state()
        print("[INTRUDER] Game stopped, returning to waiting")

    def _reset_game(self):
        self.state = "WAITING_FOR_GAME_START"
        self.robot.stop_all()
        self._publish_state()
        print("[INTRUDER] Game reset")

    def _handle_game_over(self, payload):
        if "guard" in payload:
            self.state = "LOSE"
        else:
            self.state = "WIN"

        self.robot.stop_all()
        self._publish_event(self.state)
        self._publish_state()
        print(f"[INTRUDER] Game over: {self.state}")

    # ---------------------------------------------------------
    # PUBLISH HELPERS
    # ---------------------------------------------------------
    def _publish_state(self):
        topic = f"game/intruder/{self.id}/state"
        self.comm.publish(topic, self.state)

    def _publish_event(self, event):
        topic = f"game/intruder/{self.id}/events"
        self.comm.publish(topic, event)

    # ---------------------------------------------------------
    # MAIN LOOP (PLAYER CONTROL)
    # ---------------------------------------------------------
    def update(self, controller_input):
        """
        Call this every frame with the latest Bluetooth controller input.
        """
        if self.state != "ACTIVE_PLAYER_CONTROL":
            return

        # Example: controller_input = {"x": 0.2, "y": -0.8}
        self.robot.drive(controller_input)
