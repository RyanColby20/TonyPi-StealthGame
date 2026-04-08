# tonypi_controller.py

import time
import threading

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
        Runs a high-level script (patrol, follow, chase, etc.)
        You can map script names to functions here.
        """
        print(f"[TonyPiController] Running script: {script_name}")

        script = self._get_script(script_name)
        if script is None:
            print(f"[TonyPiController] Unknown script: {script_name}")
            return

        with self._lock:
            self._stop_flag = False

        try:
            script()
        except Exception as e:
            print(f"[TonyPiController] ERROR running script: {e}")

    def _get_script(self, name):
        """
        Map script names to actual functions.
        Replace these with your real TonyPi behaviors.
        """
        scripts = {
            "patrol": self._script_patrol,
            "follow": self._script_follow,
            "chase": self._script_chase,
        }
        return scripts.get(name)

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

    # ---------------- INTERNAL ---------------- #

    def _stop_requested(self):
        with self._lock:
            return self._stop_flag
