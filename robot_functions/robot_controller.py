# tonypi_controller.py

import time
import threading
import os
import sys
import subprocess

# These imports depend on your TonyPi SDK structure.
# Adjust as needed.
import HiwonderSDK.ActionGroupControl as AGC

class TonyPiController:
    def __init__(self):
        self._stop_flag = False
        self._lock = threading.Lock()

    # ---------------- ACTION GROUPS ---------------- #

    def run_action_group(self, group_name, repeat=1):
        """
        Runs a TonyPi action group (blocking).
        RobotRole will call this inside a thread.
        """
        with self._lock:
            self._stop_flag = False

        print(f"[TonyPiController] Running action group: {group_name}")

        try:
            AGC.runActionGroup(group_name)
        except Exception as e:
            print(f"[TonyPiController] ERROR running action group: {e}")

    # ---------------- SCRIPTS ---------------- #

    def run_script(self, script_name):
        """
        Runs a high-level script (patrol, follow, chase, intruder, guard, etc.)
        If the script name is not a built-in, we attempt to run a Python file
        based on a filepath convention.
        """
        print(f"[TonyPiController] Running script: {script_name}")

        script = self._get_script(script_name)

        with self._lock:
            self._stop_flag = False

        if script is not None:
            # Built-in Python function
            try:
                script()
            except Exception as e:
                print(f"[TonyPiController] ERROR running script: {e}")
            return

        # Otherwise attempt to run a file-based script
        try:
            self._run_external_script(script_name)
        except Exception as e:
            print(f"[TonyPiController] ERROR running external script '{script_name}': {e}")

    def _get_script(self, name):
        """
        Map script names to actual functions.
        If not found, return None so run_script() falls back to file execution.
        """
        scripts = {
            "patrol": self._script_patrol,
            "follow": self._script_follow,
            "chase": self._script_chase,
            "intruder": lambda: self._run_external_script("intruder"),
            "guard": lambda: self._run_external_script("guard"),
        }
        return scripts.get(name)

    def _run_external_script(self, name):
        """
        Executes a Python script located at:
            /robot_functions/<name>/<name>_main.py

        Example:
            intruder -> /robot_functions/intruder/intruder_main.py
            guard    -> /robot_functions/guard/guard_main.py
        """
        base_dir = "/robot_functions"
        script_path = os.path.join(base_dir, name, f"{name}_main.py")

        if not os.path.isfile(script_path):
            raise FileNotFoundError(f"Script file not found: {script_path}")

        print(f"[TonyPiController] Executing external script: {script_path}")

        # Use subprocess so the script runs independently
        subprocess.Popen([sys.executable, script_path])

    # ---------------- STOP EVERYTHING ---------------- #

    def stop_all(self):
        """
        Immediately stop all robot activity.
        Called when MQTT sends a /stop command.
        """
        print("[TonyPiController] STOP ALL requested")

        with self._lock:
            self._stop_flag = True

        try:
            AGC.stopAction()
        except:
            pass

    # ---------------- EXAMPLE SCRIPTS ---------------- #

    def _script_patrol(self):
        print("[TonyPiController] Starting PATROL script")
        for _ in range(4):
            if self._stop_requested():
                return
            AGC.runActionGroup("walk")
            time.sleep(0.5)

    def _script_follow(self):
        print("[TonyPiController] Starting FOLLOW script")
        for _ in range(10):
            if self._stop_requested():
                return
            AGC.runActionGroup("look_left")
            time.sleep(0.3)
            AGC.runActionGroup("look_right")
            time.sleep(0.3)

    def _script_chase(self):
        print("[TonyPiController] Starting CHASE script")
        for _ in range(6):
            if self._stop_requested():
                return
            AGC.runActionGroup("run")
            time.sleep(0.2)

    # ---------------------------------------------------------
    # DRIVE FUNCTION FOR PLAYER CONTROL
    # ---------------------------------------------------------
    def drive(self, input_data):
        """
        input_data: {"x": float, "y": float}
        x = left/right turn (-1 to 1)
        y = forward/backward (-1 to 1)
        """

        if input_data is None:
            self.stop_all()
            return

        x = input_data.get("x", 0)
        y = input_data.get("y", 0)

        # Deadzone to prevent jitter
        if abs(x) < 0.15:
            x = 0
        if abs(y) < 0.15:
            y = 0

        # If no movement → stop
        if x == 0 and y == 0:
            if self.last_motion != "STOP":
                self.stop_all()
                self.last_motion = "STOP"
            return

        # Determine motion type
        if y > 0:
            motion = "FORWARD"
        elif y < 0:
            motion = "BACKWARD"
        elif x > 0:
            motion = "TURN_RIGHT"
        elif x < 0:
            motion = "TURN_LEFT"
        else:
            motion = "STOP"

        # Prevent spamming the same action group
        now = time.time()
        if motion == self.last_motion and (now - self.last_time) < 0.25:
            return

        self.last_motion = motion
        self.last_time = now

        # -----------------------------------------------------
        # MAP MOTION TO ACTION GROUPS
        # -----------------------------------------------------
        if motion == "FORWARD":
            # You can tune speed by selecting different action groups
            self.run_action_group("go_forward")

        elif motion == "BACKWARD":
            self.run_action_group("go_backward")

        elif motion == "TURN_RIGHT":
            self.run_action_group("turn_right")

        elif motion == "TURN_LEFT":
            self.run_action_group("turn_left")

        else:
            self.stop_all()


    # ---------------- INTERNAL ---------------- #

    def _stop_requested(self):
        with self._lock:
            return self._stop_flag
