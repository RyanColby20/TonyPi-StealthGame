# guard_main.py
"""
Fully independent guard state machine.
Run this file directly on the robot.
"""

import time
import signal
import sys

from robot_functions.robot_comm import RobotComm
from robot_functions.guard.visualPatrol import Visual_Patrol_cls
from robot_functions.guard.Follow import Follow_cls 
from robot_functions.robot_controller import TonyPiController

from robot_functions.game.robot_connection import RobotConnectionHandler



class GuardMain:
    def __init__(self, guard_id, broker_ip, robot_controller):

        self.id = guard_id
        self.robot = robot_controller

        # External behavior modules
        self.visual_patrol = Visual_Patrol_cls(robot_controller)
        self.follow_intruder = Follow_cls(robot_controller)

        # FSM state
        self.state = "WAITING_FOR_GAME_START"
        self.running = True

        # MQTT client
        self.comm = RobotComm(
            client_name=f"guard_{guard_id}",
            broker_ip=broker_ip,
            subscriptions=[
                "game/system/start",
                "game/system/stop",
                "game/system/reset",
                "game/system/game_over",
                f"game/guard/{self.id}/command"
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

        # System-wide lifecycle events
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

        # Robot-specific commands
        if topic == f"game/guard/{self.id}/command":
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
            self.state = "VISUAL_PATROL"
            self._publish_state()
            print("[GUARD] Game started → entering visual patrol")

    def _stop_game(self):
        self.state = "WAITING_FOR_GAME_START"
        self.robot.stop_all()
        self._publish_state()
        print("[GUARD] Game stopped → waiting")

    def _reset_game(self):
        self.state = "WAITING_FOR_GAME_START"
        self.robot.stop_all()
        self._publish_state()
        print("[GUARD] Game reset")

    def _trigger_game_over(self):
        print("[GUARD] Intruder captured → triggering game over")
        self.comm.publish("game/system/game_over", f"guard_{self.id}")
        self.state = "GAME_OVER"
        self._publish_state()

    # ---------------------------------------------------------
    # PUBLISH HELPERS
    # ---------------------------------------------------------
    def _publish_state(self):
        self.comm.publish(f"game/guard/{self.id}/state", self.state)

    def _publish_event(self, event):
        self.comm.publish(f"game/guard/{self.id}/events", event)

    # ---------------------------------------------------------
    # STATE LOGIC
    # ---------------------------------------------------------
    def _state_visual_patrol(self, dt):
        """
        Delegates all patrol logic to VisualPatrol.
        Returns one of:
            - "continue"
            - "intruder_detected"
        """
        result = self.visual_patrol.update(dt)

        if result == "intruder_detected":
            print("[GUARD] Intruder detected → switching to FOLLOW")
            self.state = "FOLLOW"
            self._publish_state()


    def _state_follow(self, dt):
        """
        Delegates all chase logic to FollowIntruder.
        Returns one of:
            - "continue"
            - "intruder_captured"
            - "lost_intruder"
        """
        result = self.follow_intruder.update(dt)

        if result == "intruder_captured":
            print("[GUARD] Intruder captured → triggering game over")
            self._trigger_game_over()
            return

        if result == "lost_intruder":
            print("[GUARD] Lost intruder → returning to patrol")
            self.state = "VISUAL_PATROL"
            self._publish_state()


    def _state_game_over(self):
        """
        Perform any game-over actions, then wait.
        """
        self.robot.stop_all()
        time.sleep(1.0)
        self.state = "WAITING_FOR_GAME_START"
        self._publish_state()
        print("[GUARD] Game over actions complete → waiting")


    def _enter_game_over(self):
        self.state = "GAME_OVER"
        self.robot.stop_all()
        self._publish_state()
        print("[GUARD] Entering GAME_OVER state")

    # ---------------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------------
    def run(self, fps=30):
        print("[GUARD] Running main loop...")
        frame_delay = 1.0 / fps
        last_time = time.time()

        while self.running:
            now = time.time()
            dt = now - last_time
            last_time = now

            if self.state == "VISUAL_PATROL":
                self._state_visual_patrol(dt)

            elif self.state == "FOLLOW":
                self._state_follow(dt)

            elif self.state == "GAME_OVER":
                self._state_game_over()

            time.sleep(frame_delay)

        print("[GUARD] Shutdown complete.")

    def shutdown(self):
        print("[GUARD] Shutting down...")
        self.running = False
        self.robot.stop_all()
        self.comm.disconnect()


# ---------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------
def main():
  
    connector = RobotConnectionHandler(role="guard")
    guard_id, broker_ip = connector.register_and_wait_for_id()

    robot = TonyPiController()

    guard = GuardMain(
        guard_id=guard_id,
        broker_ip=broker_ip,
        robot_controller=robot
    )

    # Handle CTRL-C
    def handle_sigint(sig, frame):
        guard.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    guard.run()


if __name__ == "__main__":
    main()
