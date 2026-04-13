import time
import paho.mqtt.client as mqtt
from robot_functions.robot_comm import RobotComm

# Import behavior modules
import visualPatrol
import Follow

BROKER_IP = "192.168.1.50"   # laptop broker

# ---------------- MQTT CALLBACK ---------------- #

def on_guard_message(client, userdata, msg):
    print("Guard received:", msg.topic, msg.payload.decode())

# ------------------------------------------------ #

def main():
    comm = RobotComm(
        client_name="guard",
        broker_ip=BROKER_IP,
        subscriptions=["intruder/status"],
        on_message=on_guard_message
    )

    comm.connect()
    print("Guard starting patrol...")

    while True:
        detected, head_position = visualPatrol.run_patrol()

        if detected:
            comm.publish("guard/detected", "intruder spotted")
            Follow.run_follow(start_position=head_position)

        time.sleep(0.1)

if __name__ == "__main__":
    main()
