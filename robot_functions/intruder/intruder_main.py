# intruder_main.py

import time
from robot_comm import RobotComm

import intruderBehavior

BROKER_IP = "192.168.1.50"

# ---------------- MQTT CALLBACK ---------------- #

def on_intruder_message(client, userdata, msg):
    payload = msg.payload.decode()
    print("Intruder received:", msg.topic, payload)

    if msg.topic == "guard/detected":
        intruderBehavior.react_to_detection()
        userdata.publish("intruder/status", "evading")

# ---------------- MAIN ---------------- #

def main():
    # Create communication object
    comm = RobotComm(
        client_name="intruder",
        broker_ip=BROKER_IP,
        subscriptions=["guard/detected"],
        on_message=on_intruder_message
    )

    # Pass comm into userdata so callback can publish
    comm.client.user_data_set(comm)

    comm.connect()
    print("Intruder starting behavior...")

    while True:
        intruderBehavior.update()
        time.sleep(0.1)

if __name__ == "__main__":
    main()
