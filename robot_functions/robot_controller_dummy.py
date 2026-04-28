# tonypi_controller_dummy.py
# A safe, hardware-free dummy TonyPiController for testing.

import time
import threading
import subprocess
import sys
from pathlib import Path

# Fake AGC stub (in case the real one isn't loaded)
try:
    import HiwonderSDK.ActionGroupControl as AGC
except Exception:
    class AGC:
        @staticmethod
        def runActionGroup(name):
            print(f"[FAKE AGC] runActionGroup('{name}')")

        @staticmethod
        def stopAction():
            print("[FAKE AGC] stopAction()")


class TonyPiController:
    def __init__(self):
        self._stop_flag = False
        self._lock = threading.Lock()
        print("[DummyTonyPi] Controller initialized")

    # ---------------- ACTION GROUPS ---------------- #

    def run_action_group(self, group_name, repeat=1):
        with self._lock:
            self._stop_flag = False

        print(f"[DummyTonyPi] run_action_group('{group_name}', repeat={repeat})")

        try:
            AGC.runActionGroup(group_name)
        except Exception as e:
            print(f"[DummyTonyPi] ERROR running action group: {e}")

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
            "intruder": lambda: self._run_external_script("robot_functions.intruder.intruder_main"),
            "guard": lambda: self._run_external_script("robot_functions.guard.guard_main"),
        }
        return scripts.get(name)

    def _run_external_script(self, module_name):
        """
        Executes a Python module like:
            robot_functions.intruder.intruder_main
        """
        print(f"[TonyPiController] Executing module: {module_name}")

        subprocess.Popen(
            [sys.executable, "-m", module_name],
            cwd=str(Path.cwd())
        )

    # ---------------- STOP EVERYTHING ---------------- #

    def stop_all(self):
        print("[DummyTonyPi] STOP ALL requested")

        with self._lock:
            self._stop_flag = True

        try:
            AGC.stopAction()
        except Exception:
            pass

    # ---------------- EXAMPLE SCRIPTS ---------------- #

    def _script_patrol(self):
        print("[DummyTonyPi] PATROL script start")
        for i in range(4):
            if self._stop_requested():
                print("[DummyTonyPi] PATROL aborted")
                return
            print(f"[DummyTonyPi] PATROL step {i+1}")
            AGC.runActionGroup("walk")
            time.sleep(0.2)

    def _script_follow(self):
        print("[DummyTonyPi] FOLLOW script start")
        for i in range(6):
            if self._stop_requested():
                print("[DummyTonyPi] FOLLOW aborted")
                return
            print(f"[DummyTonyPi] FOLLOW scan {i+1}")
            AGC.runActionGroup("look_left")
            AGC.runActionGroup("look_right")
            time.sleep(0.2)

    def _script_chase(self):
        print("[DummyTonyPi] CHASE script start")
        for i in range(5):
            if self._stop_requested():
                print("[DummyTonyPi] CHASE aborted")
                return
            print(f"[DummyTonyPi] CHASE burst {i+1}")
            AGC.runActionGroup("run")
            time.sleep(0.15)

    # ---------------- INTERNAL ---------------- #

    def _stop_requested(self):
        with self._lock:
            return self._stop_flag
