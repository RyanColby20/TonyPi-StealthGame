#!/usr/bin/python3
# coding=utf8

from Gamepad import Gamepad
from robot_functions.robot_controller_dummy import TonyPiController   # whatever class wraps run_action(), stop_actions(), etc.

def main():
    robot = TonyPiController()              # your robot control class
    pad = Gamepad(robot)         # create gamepad controller
    pad.start()                  # start background thread

    print("Gamepad control running. Press Ctrl+C to exit.")

    try:
        while True:
            pass                 # keep main thread alive
    except KeyboardInterrupt:
        print("Stopping...")
        pad.stop()

if __name__ == "__main__":
    main()
