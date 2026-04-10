import time
import threading

class TonyPiController:
    def __init__(self):
        self._stop_flag = False
        self._lock = threading.Lock()

    def run_action_group(self, group_name, repeat=1):
        print(f"[MOCK] Running action group: {group_name} (repeat={repeat})")
        for i in range(repeat):
            if self._stop_requested():
                print("[MOCK] Action group interrupted")
                return
            time.sleep(1)
        print("[MOCK] Action group complete")

    def run_script(self, script_name):
        print(f"[MOCK] Running script: {script_name}")
        for i in range(5):
            if self._stop_requested():
                print("[MOCK] Script interrupted")
                return
            print(f"[MOCK] Script step {i+1}/5")
            time.sleep(1)
        print("[MOCK] Script complete")

    def stop_all(self):
        print("[MOCK] STOP ALL received")
        with self._lock:
            self._stop_flag = True

    def _stop_requested(self):
        with self._lock:
            return self._stop_flag
        
    def run_action(self, name, times=1):
        """
        Wrapper so Gamepad system can call actions.
        """
        self.run_action_group(name, repeat=times)

    def stop_actions(self):
        """
        Wrapper so Gamepad system can stop actions.
        """
        self.stop_all()

