# intruder_main.py
"""
Intruder state machine:
- IDLE: waiting for game start
- RUNNING: joystick control active
- GAME_OVER: robot frozen
"""

import json
import time
import signal
import sys
import threading

from robot_functions.robot_comm import RobotComm
from robot_functions.robot_controller import TonyPiController
import robot_functions.intruder.Joystick as joy
from robot_functions.game.robot_connection import RobotConnectionHandler


STATE_IDLE = "IDLE"
STATE_RUNNING = "RUNNING"
STATE_GAME_OVER = "GAME_OVER"


class intruder_main:
    def __init__(self, intruder_id, broker_ip, robot_controller):
        self.id = intruder_id
        self.robot = robot_controller

        self.state = STATE_IDLE
        self.running = True
        self.pending_commands = []

        self.joystick_thread = None
        self.joystick_running = False


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
        self.enter_idle()

    # ---------------------------------------------------------
    # MQTT HANDLER
    # ---------------------------------------------------------
    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        # System events
        if topic == "game/system/start":
            self.pending_commands.append("start")
            return

        if topic == "game/system/stop":
            self.pending_commands.append("stop")
            return

        if topic == "game/system/reset":
            self.pending_commands.append("reset")
            return

        if topic == "game/system/game_over":
            self.pending_commands.append("game_over")
            return

        # Direct intruder commands
        if topic == f"game/intruder/{self.id}/command":
            self.pending_commands.append(payload)

    # ---------------------------------------------------------
    # STATE TRANSITIONS
    # ---------------------------------------------------------
    def enter_idle(self):
        self.state = STATE_IDLE
        self.robot.stop_all()
        self._stop_joystick()
        self._publish_state()
        print("[INTRUDER] Entering IDLE state, waiting for game start...")

    def enter_running(self):
        self.state = STATE_RUNNING
        self._publish_state()
        print("[INTRUDER] Game started → RUNNING (joystick enabled)")
        self._start_joystick()


    def enter_game_over(self):
        self.state = STATE_GAME_OVER
        self._stop_joystick()
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
    # Joy helpers
    # ---------------------------------------------------------

    def _start_joystick(self):
        if self.joystick_running:
            return

        self.joystick_running = True

        def run_js():
            try:
                joy.main()   # blocking call
            except Exception as e:
                print("[INTRUDER] Joystick error:", e)
            finally:
                self.joystick_running = False

        self.joystick_thread = threading.Thread(target=run_js, daemon=True)
        self.joystick_thread.start()
        print("[INTRUDER] Joystick thread started")

    def _stop_joystick(self):
        # joystick_main() must internally stop when game ends   
        try:
            joy.stop()
            self.joystick_running = False
        except:
            pass
        print("[INTRUDER] Joystick stop requested")


    # ---------------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------------

    def update(self):
        # Nothing to do here anymore
        return


    def run(self, fps=30):
        print("[INTRUDER] Main loop running...")
        frame_delay = 1.0 / fps

        while self.running:
            # Process pending commands
            if self.pending_commands:
                cmd = self.pending_commands.pop(0)
                self._handle_command(cmd)

            time.sleep(frame_delay)

        print("[INTRUDER] Shutdown complete.")

    # ---------------------------------------------------------
    # COMMAND HANDLER
    # ---------------------------------------------------------
    def _handle_command(self, cmd):
        if cmd == "start":
            self.enter_running()

        elif cmd == "stop":
            self.enter_idle()

        elif cmd == "reset":
            self.enter_idle()

        elif cmd == "game_over":
            self.enter_game_over()

    # ---------------------------------------------------------
    # SHUTDOWN
    # ---------------------------------------------------------
    def shutdown(self):
        print("[INTRUDER] Shutting down...")
        self.running = False
        self._stop_joystick()
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

    def handle_sigint(sig, frame):
        intruder.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    intruder.run()


if __name__ == "__main__":
    main()
