import time

from Constants import *

class ActionMapper:
    """
    Maps controller buttons to TonyPi action groups.
    """
    def __init__(self, robot, learn_mode=False):
        self.robot = robot
        self.learn_mode = learn_mode
        self.last_fire = {}
        
        self.BTN_SQUARE = BTN_SQUARE
        self.BTN_CROSS = BTN_CROSS
        self.BTN_TRIANGLE = BTN_TRIANGLE
        self.BTN_CIRCLE = BTN_CIRCLE

        # Map buttons → action names
        self.map = {
            self.BTN_SQUARE: "wave",
            self.BTN_CROSS: "bow",
            self.BTN_TRIANGLE: "twist",
            self.BTN_CIRCLE: "crouch",
        }

    # ---------------------------------------------------------
    def handle_button(self, b):
        now = time.time()
        if b in self.map:
            if (now - self.last_fire.get(b, 0)) >= self.BUTTON_COOLDOWN:
                self.last_fire[b] = now
                action = self.map[b]
                self.robot.run_action(action, times=1)
