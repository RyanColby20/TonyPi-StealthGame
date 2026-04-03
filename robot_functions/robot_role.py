# robot_role.py

import threading
import time

class RobotRole:
    def __init__(self, comm, robot_controller):
        """
        comm: RobotComm instance
        robot_controller: your TonyPi control object (ActionGroup, scripts, etc.)
        """
        self.comm = comm
        self.robot = robot_controller

        self.state = "IDLE"
        self.lock = threading.Lock()
        self.current_thread = None

    # ---------------- STATE MANAGEMENT ---------------- #

    def set_state(self, new_state):
        with self.lock:
            print(f"[ROLE] State change: {self.state} → {new_state}")
            self.state = new_state

    def is_idle(self):
        return self.state == "IDLE"

    # ---------------- COMMAND HANDLERS ---------------- #

    def handle_action_group(self, group_name):
        if not self.is_idle():
            print("[ROLE] Busy, ignoring action group")
            return

        def run():
            self.set_state("RUNNING_ACTION")
            self.robot.run_action_group(group_name)
            self.set_state("IDLE")

        self.current_thread = threading.Thread(target=run, daemon=True)
        self.current_thread.start()

    def handle_script(self, script_name):
        if not self.is_idle():
            print("[ROLE] Busy, ignoring script")
            return

        def run():
            self.set_state("RUNNING_SCRIPT")
            self.robot.run_script(script_name)
            self.set_state("IDLE")

        self.current_thread = threading.Thread(target=run, daemon=True)
        self.current_thread.start()

    def stop_current(self):
        print("[ROLE] Stop requested")
        self.robot.stop_all()
        self.set_state("IDLE")
