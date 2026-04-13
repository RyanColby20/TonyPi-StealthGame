import time
import tkinter as tk
from tkinter import ttk
from broker_controller import BrokerController

BROKER_IP = "localhost"

BUTTON_ACTIONS = {
    "Stand": ["stand"],
    "Wave": ["wave"],
    "Kick": ["kick"],
    "Sit": ["sit"],
    "Action 5": ["bow"],
    "Action 6": ["sit"],
    "Action 7": ["demo_sequence"],
    "Action 8": ["pushup"],
    "Action 9": ["walk_forward"],
    "Action 10": ["walk_backward"],
    "Action 11": ["turn_left"],
    "Action 12": ["turn_right"]
}


# ============================================================
#  MONITOR WINDOW (Log + Heartbeat)
# ============================================================
class MonitorWindow:
    def __init__(self, root):
        self.win = tk.Toplevel(root)
        self.win.title("Robot Monitor")
        self.win.geometry("600x800")

        # Log
        ttk.Label(self.win, text="Log", font=("Arial", 18, "bold")).pack(pady=10)
        self.log = tk.Text(self.win, width=60, height=20, state="disabled")
        self.log.pack(padx=10, pady=5)

        # Heartbeat
        ttk.Label(self.win, text="Heartbeat", font=("Arial", 18, "bold")).pack(pady=10)
        self.heartbeat_frame = ttk.Frame(self.win)
        self.heartbeat_frame.pack(padx=10, pady=5, anchor="w")

        self.heartbeat_labels = {}
        self.heartbeat_times = {}

    # ------------------------------
    # Logging
    # ------------------------------
    def log_msg(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    # ------------------------------
    # Heartbeat handling
    # ------------------------------
    def add_robot(self, robot_name):
        if robot_name in self.heartbeat_labels:
            return

        label = ttk.Label(self.heartbeat_frame, text=f"{robot_name}: 🔴", foreground="red")
        label.pack(anchor="w")

        self.heartbeat_labels[robot_name] = label
        self.heartbeat_times[robot_name] = 0

    def update_heartbeat(self, robot_name):
        self.heartbeat_times[robot_name] = time.time()

    def check_heartbeats(self):
        now = time.time()
        timeout = 3

        for robot, label in self.heartbeat_labels.items():
            last = self.heartbeat_times.get(robot, 0)
            alive = (now - last) < timeout

            if alive:
                label.config(text=f"{robot}: 🟢", foreground="green")
            else:
                label.config(text=f"{robot}: 🔴", foreground="red")

        self.win.after(400, self.check_heartbeats)


# ============================================================
#  MAIN DEMO GUI
# ============================================================
class DemoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TonyPi Demo Controller")

        # FULL SCREEN
        self.root.attributes("-fullscreen", True)
        self.root.bind("<Escape>", lambda e: self.root.attributes("-fullscreen", False))

        # ttk theme + button style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Demo.TButton",
            font=("Arial", 20, "bold"),
            padding=20
        )

        style = ttk.Style()

        # ------------------------------
        # TTK STYLING 
        # ------------------------------
        style.theme_use("clam")

        # Create a custom layout with rounded corners
        style.element_create(
            "RoundedFrame",
            "from",
            "clam"
        )

        style.layout(
            "Rounded.TButton",
            [
                ("Button.focus", {
                    "children": [
                        ("Button.padding", {
                            "children": [
                                ("Button.label", {"side": "left", "expand": 1})
                            ],
                            "sticky": "nswe"
                        })
                    ],
                    "sticky": "nswe"
                })
            ]
        )

        style.configure(
            "Rounded.TButton",
            font=("Arial", 20, "bold"),
            padding=20,
            relief="flat",
            borderwidth=0,
            background="#4A90E2",
            foreground="white"
        )

        style.map(
            "Rounded.TButton",
            background=[
                ("active", "#6BB5FF"),
                ("pressed", "#3A78C2")
            ]
        )

        # ------------------------------
        # TTK STYLING END
        # ------------------------------

        # Controller
        self.controller = BrokerController(
            client_name="demo_gui",
            broker_ip=BROKER_IP,
            on_robot_seen=self.add_robot,
            on_heartbeat=self.on_heartbeat
        )

        self.discovered_robots = set()
        self.alive_robots = set()

        # Create monitor window
        self.monitor = MonitorWindow(root)
        self.monitor.check_heartbeats()

        # Build main layout
        self._build_layout()

    # ============================================================
    #  GUI LAYOUT
    # ============================================================
    def _build_layout(self):
        # Configure root grid
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=3)
        self.root.columnconfigure(2, weight=1)

        # Left panel: robot selection
        left = ttk.Frame(self.root, padding=20)
        left.grid(row=0, column=0, sticky="ns")

        ttk.Label(left, text="Robots", font=("Arial", 22, "bold")).pack(pady=10)

        ttk.Button(left, text="Select All", command=self.select_all).pack(pady=5)
        ttk.Button(left, text="Clear All", command=self.clear_all).pack(pady=5)

        self.robot_listbox = tk.Listbox(left, selectmode=tk.MULTIPLE, height=12, font=("Arial", 16))
        self.robot_listbox.pack(fill="both", expand=True, pady=10)

        # Middle panel: 12-button grid
        mid = ttk.Frame(self.root, padding=20)
        mid.grid(row=0, column=1, sticky="nsew")
        self._build_action_buttons(mid)

        # Right panel: STOP button
        right = ttk.Frame(self.root, padding=20)
        right.grid(row=0, column=2, sticky="ns")

        ttk.Button(
            right,
            text="STOP",
            style="Demo.TButton",
            command=self.stop_selected
        ).pack(pady=40)

    # ============================================================
    #  12-BUTTON GRID
    # ============================================================
    def _build_action_buttons(self, parent):
        ttk.Label(parent, text="Demo Actions", font=("Arial", 28, "bold")).pack(pady=20)

        grid = ttk.Frame(parent)
        grid.pack(expand=True, fill="both")

        labels = list(BUTTON_ACTIONS.keys())

        rows = 4
        cols = 3
        idx = 0

        for r in range(rows):
            grid.rowconfigure(r, weight=1)
            for c in range(cols):
                grid.columnconfigure(c, weight=1)

                label = labels[idx]
                action_list = BUTTON_ACTIONS[label]
                idx += 1

                btn = ttk.Button(
                    grid,
                    text=label,
                    style="Rounded.TButton",
                    command=lambda al=action_list: self.run_action_group(al)
                )
                btn.grid(row=r, column=c, padx=20, pady=20, sticky="nsew")

    # ============================================================
    #  Utility
    # ============================================================
    def log_msg(self, msg):
        self.monitor.log_msg(msg)

    def selected_robots(self):
        indices = self.robot_listbox.curselection()
        return [self.robot_listbox.get(i) for i in indices]

    def select_all(self):
        self.robot_listbox.select_set(0, tk.END)

    def clear_all(self):
        self.robot_listbox.select_clear(0, tk.END)

    # ============================================================
    #  Commands
    # ============================================================
    def run_action_group(self, action_list):
        robots = self.selected_robots()
        if not robots:
            return

        num_actions = len(action_list)

        for i, robot in enumerate(robots):
            action_group = action_list[i % num_actions]
            self.controller.run_action_group([robot], action_group)
            self.log_msg(f"Sent '{action_group}' to {robot}")

    def stop_selected(self):
        robots = self.selected_robots()
        if robots:
            self.controller.stop_robots(robots)
            self.log_msg(f"STOP sent to {robots}")

    # ============================================================
    #  Robot Discovery + Heartbeat
    # ============================================================
    def add_robot(self, robot_name):
        self.root.after(0, lambda: self._add_robot_safe(robot_name))

    def _add_robot_safe(self, robot_name):
        if robot_name in self.discovered_robots:
            return

        self.discovered_robots.add(robot_name)
        self.robot_listbox.insert(tk.END, robot_name)
        self.alive_robots.add(robot_name)

        self.monitor.add_robot(robot_name)
        self.log_msg(f"Discovered robot: {robot_name}")

    def on_heartbeat(self, robot_name):
        self.monitor.update_heartbeat(robot_name)


# ============================================================
#  MAIN
# ============================================================
if __name__ == "__main__":
    root = tk.Tk()
    gui = DemoGUI(root)
    root.mainloop()
