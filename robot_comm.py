# robot_comm.py

import paho.mqtt.client as mqtt
import time
import threading 


class RobotComm:
    def __init__(self, client_name, broker_ip, subscriptions=None, on_message=None, heartbeat_interval=2.0):
        self.client_name = client_name
        self.broker_ip = broker_ip
        self.subscriptions = subscriptions or []
        self.on_message_callback = on_message
        self.heartbeat_interval = heartbeat_interval

        self.client = mqtt.Client(client_name)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = on_message

        self.connected = False
        self._stop_flag = False

    # ---------------- MQTT CALLBACKS ---------------- #

    def _on_connect(self, client, userdata, flags, rc):
        self.connected = True
        print(f"[{self.client_name}] Connected to MQTT with code {rc}")

        # Re-subscribe after reconnect
        for topic in self.subscriptions:
            client.subscribe(topic)
            print(f"[{self.client_name}] Subscribed to {topic}")

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        print(f"[{self.client_name}] Disconnected from MQTT (rc={rc})")

        # Start reconnection attempts in a background thread
        threading.Thread(target=self._reconnect_loop, daemon=True).start()

    # ---------------- CONNECTION MANAGEMENT ---------------- #

    def connect(self):
        """Initial connection + start heartbeat thread."""
        self.client.connect(self.broker_ip, 1883, 60)
        self.client.loop_start()

        # Start heartbeat thread
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    def _reconnect_loop(self):
        """Try reconnecting with exponential backoff."""
        delay = 1
        while not self.connected and not self._stop_flag:
            try:
                print(f"[{self.client_name}] Attempting reconnect...")
                self.client.reconnect()
                return
            except:
                print(f"[{self.client_name}] Reconnect failed, retrying in {delay}s")
                time.sleep(delay)
                delay = min(delay * 2, 30)  # cap at 30 seconds

    # ---------------- HEARTBEAT ---------------- # 

    def _heartbeat_loop(self):
        """Send periodic heartbeat messages."""
        while not self._stop_flag:
            if self.connected:
                topic = f"robot/{self.client_name}/heartbeat"
                payload = str(time.time())
                self.client.publish(topic, payload)
            time.sleep(self.heartbeat_interval)


    # ---------------- PUBLISH ---------------- #

    def publish(self, topic, message):
        if self.connected:
            self.client.publish(topic, message)
        else:
            print(f"[{self.client_name}] WARNING: Tried to publish while disconnected.")

    # ---------------- CLEANUP ---------------- #

    def stop(self):
        self._stop_flag = True
        self.client.loop_stop()
        self.client.disconnect()