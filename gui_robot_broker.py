# gui_broker.py

import time
import tkinter as tk
from tkinter import ttk
from broker_controller import BrokerController

BROKER_IP = "localhost" # 192.168.137.1 but localhost should always be correct
ROBOT_NAMES = ["alpha", "bravo", "charlie"]   # or load from config

class BrokerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TonyPi Broker Controller")

        self.controller = BrokerController(
            client_name="broker_gui",
            broker_ip=BROKER_IP,
            on_robot_seen=self.add_robot,
            on_heartbeat=self.on_heartbeat
        )
        
        self.discovered_robots = set()
        self.alive_robots = set()
        
        self._build_layout()        
        
        self.heartbeat_times = {}
        self.heartbeat_labels = {}
        self._start_heartbeat_checker()

    # ---------------------------------------------------------
    # GUI LAYOUT
    # ---------------------------------------------------------
    def _build_layout(self):
        # Left panel: robot selection
        left = tk.Frame(self.root, padx=10, pady=10)
        left.grid(row=0, column=0, sticky="ns")

        tk.Label(left, text="Robots", font=("Arial", 14, "bold")).pack()

        tk.Button(left, text="Select All", command=self.select_all).pack(pady=5)
        tk.Button(left, text="Clear All", command=self.clear_all).pack(pady=5)

        # Middle panel: actions
        mid = tk.Frame(self.root, padx=10, pady=10)
        mid.grid(row=0, column=1, sticky="n")

        tk.Label(mid, text="Commands", font=("Arial", 14, "bold")).pack()

        # Action group
        tk.Label(mid, text="Action Group:").pack(anchor="w")
        self.action_group_entry = tk.Entry(mid)
        self.action_group_entry.pack(fill="x")

        tk.Button(mid, text="Run Action Group", command=self.run_action_group).pack(fill="x", pady=5)

        # Script
        tk.Label(mid, text="Script:").pack(anchor="w")
        self.script_entry = tk.Entry(mid)
        self.script_entry.pack(fill="x")

        tk.Button(mid, text="Run Script", command=self.run_script).pack(fill="x", pady=5)

        # Stop
        tk.Button(mid, text="STOP Selected Robots", bg="red", fg="white",
                  command=self.stop_selected).pack(fill="x", pady=10)

        # Right panel: log
        right = tk.Frame(self.root, padx=10, pady=10)
        right.grid(row=0, column=2, sticky="nsew")

        tk.Label(right, text="Log", font=("Arial", 14, "bold")).pack()

        self.log = tk.Text(right, width=50, height=25, state="disabled")
        self.log.pack()
        
        # Heartbeat panel INSIDE the commands panel
        hb_frame = tk.Frame(mid, padx=10, pady=10)
        hb_frame.pack(side="bottom", anchor="s", fill="x")

        tk.Label(hb_frame, text="Heartbeat Monitor", font=("Arial", 12, "bold")).pack(anchor="w")

        self.heartbeat_frame = tk.Frame(hb_frame)
        self.heartbeat_frame.pack(anchor="w")
        
        # Listbox for robot selection
        self.robot_listbox = tk.Listbox(left, selectmode=tk.MULTIPLE, height=8)
        self.robot_listbox.pack(fill="both", expand=True)


    # ---------------------------------------------------------
    # Utility
    # ---------------------------------------------------------
    def log_msg(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def selected_robots(self):
        indices = self.robot_listbox.curselection()
        return [self.robot_listbox.get(i) for i in indices]
    
    def select_all(self):
        self.robot_listbox.select_set(0, tk.END)

    def clear_all(self):
        self.robot_listbox.select_clear(0, tk.END)


    # ---------------------------------------------------------
    # Command Handlers
    # ---------------------------------------------------------
    def run_action_group(self):
        robots = self.selected_robots()
        group = self.action_group_entry.get().strip()
        if robots and group:
            self.controller.run_action_group(robots, group)
            self.log_msg(f"Action group '{group}' sent to {robots}")

    def run_script(self):
        robots = self.selected_robots()
        script = self.script_entry.get().strip()
        if robots and script:
            self.controller.run_script(robots, script)
            self.log_msg(f"Script '{script}' sent to {robots}")

    def stop_selected(self):
        robots = self.selected_robots()
        if robots:
            self.controller.stop_robots(robots)
            self.log_msg(f"STOP sent to {robots}")

    # ---------------------------------------------------------
    # Dynamic robot discovery
    # ---------------------------------------------------------
    def add_robot(self, robot_name):
        self.root.after(0, lambda: self._add_robot_safe(robot_name))

    def _add_robot_safe(self, robot_name):

        # Only create GUI elements once per robot
        if robot_name in self.discovered_robots:
            return

        self.discovered_robots.add(robot_name)

        # Create heartbeat label ONCE
        hb_label = tk.Label(self.heartbeat_frame, text=f"{robot_name}: 🔴", fg="red")
        hb_label.pack(anchor="w")
        self.heartbeat_labels[robot_name] = hb_label
        self.heartbeat_times[robot_name] = 0

        # Add robot to listbox as alive
        self.robot_listbox.insert(tk.END, robot_name)
        self.alive_robots.add(robot_name)

        self.log_msg(f"Discovered robot: {robot_name}")
        
    def on_heartbeat(self, robot_name):
        self.heartbeat_times[robot_name] = time.time()
    
    def _start_heartbeat_checker(self):
        self._check_heartbeats()
        
    def _check_heartbeats(self):
        now = time.time()
        timeout = 3
        # Currently, robots appear to send heartbeat messages roughly every 2 seconds, so 2.5 was selected
        # because it was slightly above it. Prevents false positives while notifying of disconnects ASAP        

        dead_robots = []
        for robot, label in self.heartbeat_labels.items():
            last = self.heartbeat_times.get(robot, 0)
            alive = (now - last) < timeout
            
            if alive:
                label.config(text=f"{robot}: 🟢", fg="green")

                # If robot was dead but is now alive, re-add to listbox
                if robot not in self.alive_robots:
                    self.alive_robots.add(robot)

                    # Only insert if not already in listbox
                    if robot not in self.robot_listbox.get(0, tk.END):
                        self.robot_listbox.insert(tk.END, robot)
            else:
                label.config(text=f"{robot}: 🔴", fg="red")
                dead_robots.append(robot)

        # Functionality for robot removal after death is below
        # Remove dead robots from listbox
        for robot in dead_robots:
            if robot in self.alive_robots:
                self.alive_robots.remove(robot)

                # Remove from listbox
                for i in range(self.robot_listbox.size()):
                    if self.robot_listbox.get(i) == robot:
                        self.robot_listbox.delete(i)
                        break

                    
        self.root.after(400, self._check_heartbeats)



# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    gui = BrokerGUI(root)
    root.mainloop()
