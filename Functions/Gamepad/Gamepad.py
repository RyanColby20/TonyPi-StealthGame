import os
import time
import threading
import pygame

from Head_Controller import HeadController
from Movement_Controller import MovementController
from Action_Mapper import ActionMapper
from Constants import *

class Gamepad:
    """
    High-level joystick controller.
    Handles pygame input and dispatches events to:
      - HeadController
      - MovementController
      - ActionMapper
    """

    def __init__(self, robot, learn_mode=False):
        self.robot = robot
        self.learn_mode = learn_mode

        self.js = None
        self.connected = False
        self.stop_flag = False

        self.prev_buttons = {}
        self.prev_hat = (0, 0)

        # Subsystems
        self.head = HeadController(robot, learn_mode=learn_mode)
        self.movement = MovementController(robot, learn_mode=learn_mode)
        self.actions = ActionMapper(robot, learn_mode=learn_mode)

        # Thread handle
        self.thread = None
        
        self.LOOP_SLEEP = LOOP_SLEEP

    # ---------------------------------------------------------
    # Public API
    # ---------------------------------------------------------
    def start(self):
        """Start the gamepad loop in a background thread."""
        if self.thread and self.thread.is_alive():
            return

        self.stop_flag = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop the loop and reset head."""
        self.stop_flag = True
        self.head.force_exit_reset()

    def run(self):
        """Blocking loop."""
        self._init_pygame()
        self.head.recenter()

        while not self.stop_flag:
            self._check_connection()
            if not self.connected:
                time.sleep(0.2)
                continue

            pygame.event.pump()
            self._handle_buttons()
            self._handle_hat()
            self._handle_look()
            self._handle_movement()

            time.sleep(self.LOOP_SLEEP)

    # ---------------------------------------------------------
    # Internal helpers
    # ---------------------------------------------------------
    def _init_pygame(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.display.init()
        pygame.joystick.init()

    def _check_connection(self):
        if os.path.exists("/dev/input/js0"):
            if not self.connected:
                self.js = pygame.joystick.Joystick(0)
                self.js.init()
                self.connected = True
                self.prev_buttons = {i: 0 for i in range(self.js.get_numbuttons())}
                if self.learn_mode:
                    print("Joystick connected:", self.js.get_name())
        else:
            if self.connected:
                self.connected = False
                try:
                    self.js.quit()
                except Exception:
                    pass
                pygame.joystick.quit()
                self.js = None
                if self.learn_mode:
                    print("Joystick disconnected")

    # ---------------------------------------------------------
    # Input handlers
    # ---------------------------------------------------------
    def _handle_buttons(self):
        for b in range(self.js.get_numbuttons()):
            current = self.js.get_button(b)
            previous = self.prev_buttons.get(b, 0)

            if current and not previous:
                # Button pressed
                self.actions.handle_button(b)

                # START button → stand + recenter
                if b == BTN_START:
                    self.robot.run_action("stand", times=1)
                    self.head.recenter()

            self.prev_buttons[b] = current

    def _handle_hat(self):
        if self.js.get_numhats() == 0:
            return

        hat = self.js.get_hat(0)
        if hat != self.prev_hat and self.learn_mode:
            print("HAT:", hat)
        self.prev_hat = hat

    def _handle_look(self):
        rx = self.js.get_axis(self.head.RX_AXIS)
        ry = self.js.get_axis(self.head.RY_AXIS)
        self.head.update(rx, ry)

    def _handle_movement(self):
        lx = self.js.get_axis(0)
        ly = self.js.get_axis(1)
        self.movement.update(lx, ly)
