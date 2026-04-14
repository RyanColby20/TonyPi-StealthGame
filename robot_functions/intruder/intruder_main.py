# intruder_main.py
'''
This file runs the state machine responsible for managing the intruder
This is the file that should be run to initialize an intruder within the game.
'''

import json
import time
import signal
import sys
import uuid

from robot_functions.robot_comm import RobotComm
from robot_functions.robot_controller import TonyPiController
from controller_input.bluetooth_input import get_controller_input

from robot_functions.game.robot_connection import RobotConnectionHandler


class intruder_main:
    def __init__(self, intruder_id, broker_ip, robot_controller):
        self.id = intruder_id
        self.robot = robot_controller

        self.state = "WAITING_FOR_GAME_START"
        self.running = True

        self.comm = RobotComm(
            client_name=f"intruder_{intruder_id}",
            broker_ip=broker_ip,
            subscriptions=[
                "game/system/start",
                "game/system/stop",
                "game/system/reset",
                "game/system/game_over",
                f"game/intruder/{self.id}/command"
            ],
            on_message=self._on_message,
            heartbeat_interval=5.0,
            heartbeat_prefix="game/robot"
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
            return

        if topic == "game/system/stop":
            self._stop_game()
            return

        if topic == "game/system/reset":
            self._reset_game()
            return

        if topic == "game/system/game_over":
            self._enter_game_over()
            return

        if topic == f"game/intruder/{self.id}/command":
            if payload == "stop":
                self._stop_game()
            elif payload == "reset":
                self._reset_game()
            elif payload == "game_over":
                self._enter_game_over()

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

    def _enter_game_over(self):
        self.state = "GAME_OVER"
        self.robot.stop_all()
        self._publish_state()
        print("[INTRUDER] Entering GAME_OVER state")

    # ---------------------------------------------------------
    # PUBLISH HELPERS
    # ---------------------------------------------------------
    def _publish_state(self):
        self.comm.publish(f"game/intruder/{self.id}/state", self.state)

    def _publish_event(self, event):
        self.comm.publish(f"game/intruder/{self.id}/events", event)

    # ---------------------------------------------------------
    # MAIN LOOP (PLAYER CONTROL)
    # ---------------------------------------------------------
    def update(self):
        """Called every frame."""
        if self.state != "ACTIVE_PLAYER_CONTROL":
            return

        controller_input = self.robot.get_input()
        if controller_input is None:
            return

        self.robot.drive(controller_input)

    def run(self, fps=30):
        """Main loop that keeps the intruder alive."""
        print("[INTRUDER] Running main loop...")
        frame_delay = 1.0 / fps

        while self.running:
            self.update()
            time.sleep(frame_delay)

        print("[INTRUDER] Shutdown complete.")

    def shutdown(self):
        """Graceful shutdown."""
        print("[INTRUDER] Shutting down...")
        self.running = False
        self.robot.stop_all()
        self.comm.disconnect()


# ---------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------
def main():


    connector = RobotConnectionHandler(role="intruder")
    intruder_id, broker_ip = connector.register_and_wait_for_id()

    robot = TonyPiController()
    intruder = intruder_main(
        intruder_id=intruder_id,
        broker_ip=broker_ip,
        robot_controller=robot
    )

    # Handle CTRL-C
    def handle_sigint(sig, frame):
        intruder.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    intruder.run()


if __name__ == "__main__":
    main()