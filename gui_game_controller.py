# gui_game_controller.py

import tkinter as tk
from game_functions.GameController import GameController
from HiwonderSDK.yaml_handle import load_robot_config



class GameGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Game Controller")
        


        config = load_robot_config()
        BROKER_IP = config.get("robot", {}).get("broker_ip", "127.0.0.1")

        self.controller = GameController(
            broker_ip=BROKER_IP,
            on_event=self.add_log,
            on_role_update=self.update_roles,
            on_disconnect=self.handle_disconnect
        )

        self.guards = {}
        self.intruders = {}

        self._build_layout()

        # start heartbeat monitor
        self.root.after(1000, self._heartbeat_monitor)

    # ---------------------------------------------------------
    # GUI LAYOUT
    # ---------------------------------------------------------
    def _build_layout(self):
        left = tk.Frame(self.root, padx=10, pady=10)
        left.grid(row=0, column=0, sticky="ns")

        tk.Label(left, text="Guards", font=("Arial", 14, "bold")).pack()
        self.guard_frame = tk.Frame(left)
        self.guard_frame.pack()

        tk.Label(left, text="Intruders", font=("Arial", 14, "bold")).pack(pady=(20,0))
        self.intruder_frame = tk.Frame(left)
        self.intruder_frame.pack()

        mid = tk.Frame(self.root, padx=10, pady=10)
        mid.grid(row=0, column=1, sticky="n")

        tk.Button(mid, text="Start Game", command=self.controller.start_game).pack(fill="x", pady=5)
        tk.Button(mid, text="Stop Game", command=self.controller.stop_game).pack(fill="x", pady=5)
        tk.Button(mid, text="Reset Game", command=self.controller.reset_game).pack(fill="x", pady=5)

        right = tk.Frame(self.root, padx=10, pady=10)
        right.grid(row=0, column=2, sticky="nsew")

        tk.Label(right, text="Event Log", font=("Arial", 14, "bold")).pack()
        self.log = tk.Text(right, width=60, height=30, state="disabled")
        self.log.pack()

    # ---------------------------------------------------------
    # CALLBACKS FROM CONTROLLER
    # ---------------------------------------------------------
    def add_log(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def update_roles(self, robot_name, role, role_id):
        if role == "guard":
            lbl = tk.Label(self.guard_frame, text=f"Guard {role_id}: {robot_name}", fg="green")
            lbl.pack(anchor="w")
            self.guards[role_id] = lbl

        elif role == "intruder":
            lbl = tk.Label(self.intruder_frame, text=f"Intruder {role_id}: {robot_name}", fg="green")
            lbl.pack(anchor="w")
            self.intruders[role_id] = lbl


    # ---------------------------------------------------------
    # INTERNAL HELPER
    # ---------------------------------------------------------
    def _heartbeat_monitor(self):
        disconnected = self.controller.check_robot_timeouts()

        self.root.after(1000, self._heartbeat_monitor)

    def handle_disconnect(self, mac, role, role_id):
        if role == "guard" and role_id in self.guards:
            lbl = self.guards[role_id]
            lbl.config(fg="red")
            self.add_log(f"[DISCONNECT] Guard {role_id} ({mac}) lost connection")

        elif role == "intruder" and role_id in self.intruders:
            lbl = self.intruders[role_id]
            lbl.config(fg="red")
            self.add_log(f"[DISCONNECT] Intruder {role_id} ({mac}) lost connection")




# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    gui = GameGUI(root)
    root.mainloop()
