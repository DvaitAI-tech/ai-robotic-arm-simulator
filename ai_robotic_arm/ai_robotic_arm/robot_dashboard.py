#!/usr/bin/env python3
"""
🤖 robotDash - DvaitAI Robotic Arm Dashboard (ROS 2 + Tkinter + Matplotlib)
Now with Pause/Resume Graph and AI Mode Indicator Light
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading
import subprocess
import signal
import sys
import os
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class robotDash(tk.Tk, Node):
    """Combined ROS2 Node + Tkinter Dashboard with live graph and indicators."""

    def __init__(self):
        # Initialize ROS
        rclpy.init(args=None)
        Node.__init__(self, 'robot_dash_node')

        # Initialize GUI
        tk.Tk.__init__(self)
        self.title("🤖 robotDash - DvaitAI Robotic Arm Dashboard")
        self.geometry("820x1020")
        self.resizable(False, False)

        # ROS Publisher & Subscriber
        self.cmd_pub = self.create_publisher(String, '/arm_command', 10)
        self.ai_stop_pub = self.create_publisher(String, '/ai_stop', 10)
        self.create_subscription(String, '/arm_status', self.arm_status_callback, 10)

        # Variables
        self.status_text = tk.StringVar(value="ROS Node Ready ✅")
        self.mode_var = tk.StringVar(value="Manual")
        self.arm_feedback = tk.StringVar(value="No updates yet.")
        self.ai_process = None

        # Graph control flags
        self.pause_graph = tk.BooleanVar(value=False)

        # Paths
        self.log_file = os.path.expanduser("arm_log.csv")

        # Build UI
        self.create_widgets()

        # ROS spin thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Graceful exit
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start graph refresh loop
        self.after(1000, self.update_graph)

    # ----------------------------------------------------------------
    # ROS SPIN THREAD
    # ----------------------------------------------------------------
    def spin_ros(self):
        try:
            rclpy.spin(self)
        except Exception as e:
            print(f"[ROS ERROR]: {e}")
        finally:
            self.destroy_node()
            rclpy.shutdown()

    # ----------------------------------------------------------------
    # GUI LAYOUT
    # ----------------------------------------------------------------
    def create_widgets(self):
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill='both', expand=True)

        # Manual controls
        control_frame = ttk.LabelFrame(main_frame, text="Manual Controls", padding="10")
        control_frame.grid(row=0, column=0, sticky='ew', pady=10)

        move_buttons = [
            ("Move Up", lambda: self.send_command("move up")),
            ("Move Down", lambda: self.send_command("move down")),
            ("Move Left", lambda: self.send_command("move left")),
            ("Move Right", lambda: self.send_command("move right")),
        ]
        for i, (label, cmd) in enumerate(move_buttons):
            ttk.Button(control_frame, text=f"[ {label} ]", command=cmd).grid(
                row=i, column=0, columnspan=2, sticky='ew', pady=3
            )

        ttk.Button(control_frame, text="[ Pick ]",
                   command=lambda: self.send_command("pick")).grid(
            row=4, column=0, sticky='ew', pady=5, padx=(0, 5))
        ttk.Button(control_frame, text="[ Drop ]",
                   command=lambda: self.send_command("drop")).grid(
            row=4, column=1, sticky='ew', pady=5, padx=(5, 0))

        # Mode + status
        status_frame = ttk.LabelFrame(main_frame, text="Mode & Status", padding="10")
        status_frame.grid(row=1, column=0, sticky='ew', pady=10)

        # --- AI Indicator Light ---
        self.ai_indicator = tk.Canvas(status_frame, width=20, height=20, highlightthickness=0)
        self.ai_light = self.ai_indicator.create_oval(3, 3, 17, 17, fill="red")
        self.ai_indicator.grid(row=0, column=3, padx=(20, 0))

        ttk.Label(status_frame, text="Mode:").grid(row=0, column=0, padx=5, pady=5, sticky='w')
        ttk.Radiobutton(status_frame, text="Manual", variable=self.mode_var,
                        value="Manual", command=self.switch_to_manual_mode).grid(row=0, column=1, padx=5)
        ttk.Radiobutton(status_frame, text="AI", variable=self.mode_var,
                        value="AI", command=self.switch_to_ai_mode).grid(row=0, column=2, padx=5)

        ttk.Label(status_frame, text="Current Status:").grid(row=1, column=0, sticky='w', padx=5)
        ttk.Label(status_frame, textvariable=self.status_text,
                  font=('Helvetica', 10, 'bold'),
                  anchor='w').grid(row=1, column=1, columnspan=3, sticky='ew', padx=5)

        # Arm feedback
        feedback_frame = ttk.LabelFrame(main_frame, text="Arm Feedback (/arm_status)", padding="10")
        feedback_frame.grid(row=2, column=0, sticky='ew', pady=10)
        ttk.Label(feedback_frame, textvariable=self.arm_feedback,
                  font=('Courier', 11, 'bold'), anchor='center').pack(fill='x')

        # ---- Graph frame ----
        graph_frame = ttk.LabelFrame(main_frame, text="Telemetry Visualization", padding="10")
        graph_frame.grid(row=3, column=0, sticky='nsew', pady=10)
        main_frame.grid_rowconfigure(3, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        # Create Matplotlib figure
        self.fig = Figure(figsize=(6.5, 3.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Arm Angles (Last 30 Actions)")
        self.ax.set_xlabel("Action Index")
        self.ax.set_ylabel("Angle (°)")
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # Pause/Resume Graph Toggle
        ttk.Checkbutton(graph_frame, text="⏸ Pause Graph",
                        variable=self.pause_graph,
                        onvalue=True, offvalue=False,
                        command=self.toggle_graph_pause).pack(anchor='e', pady=5)

        # Exit button
        ttk.Button(main_frame, text="[ Exit ]", command=self.on_close).grid(
            row=4, column=0, sticky='ew', pady=10)

    # ----------------------------------------------------------------
    # ROS COMMUNICATION
    # ----------------------------------------------------------------
    def send_command(self, key: str):
        if self.mode_var.get() == "AI":
            self.update_status("❌ Cannot send manual commands in AI mode.")
            return
        msg = String()
        msg.data = key
        self.cmd_pub.publish(msg)
        self.update_status(f"✅ Sent → {msg.data}")

    def arm_status_callback(self, msg):
        feedback = msg.data
        self.after(0, lambda: self.arm_feedback.set(f"📡 {feedback}"))

    # ----------------------------------------------------------------
    # MODE SWITCHING
    # ----------------------------------------------------------------
    def switch_to_ai_mode(self):
        self.update_status("🧠 Switching to AI Mode... Starting autopilot...")
        self.set_ai_indicator(True)
        if self.ai_process is None:
            try:
                self.ai_process = subprocess.Popen(
                    ["ros2", "run", "ai_robotic_arm", "ai_autopilot"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                )
                self.update_status("🤖 AI autopilot running...")
            except Exception as e:
                self.update_status(f"❌ Failed to start AI autopilot: {e}")
                self.mode_var.set("Manual")
                self.set_ai_indicator(False)

    def switch_to_manual_mode(self):
        self.update_status("🧩 Switching to Manual Mode...")
        self.set_ai_indicator(False)

        stop_msg = String()
        stop_msg.data = "stop"
        self.ai_stop_pub.publish(stop_msg)

        if self.ai_process:
            try:
                self.ai_process.terminate()
                self.ai_process.wait(timeout=2)
            except Exception as e:
                self.update_status(f"⚠️ Error stopping AI autopilot: {e}")
            self.ai_process = None

        self.update_status("✅ Manual mode active.")

    # ----------------------------------------------------------------
    # GRAPH UPDATE & CONTROL
    # ----------------------------------------------------------------
    def toggle_graph_pause(self):
        if self.pause_graph.get():
            self.update_status("⏸ Graph paused.")
        else:
            self.update_status("▶️ Graph resumed.")

    def update_graph(self):
        """Fetch latest CSV telemetry and refresh chart."""
        if self.pause_graph.get():
            self.after(1500, self.update_graph)
            return

        try:
            if not os.path.exists(self.log_file):
                self.ax.clear()
                self.ax.text(0.5, 0.5, "No telemetry data yet...",
                             ha='center', va='center', fontsize=10)
                self.canvas.draw()
            else:
                df = pd.read_csv(self.log_file)
                if not df.empty:
                    y1 = df["angle1"].values[-30:]
                    y2 = df["angle2"].values[-30:]
                    x = range(len(y1))
                    self.ax.clear()
                    self.ax.plot(x, y1, label="Angle 1 (Base)", linewidth=2)
                    self.ax.plot(x, y2, label="Angle 2 (Joint)", linewidth=2)
                    self.ax.set_title("Arm Angles (Last 30 Actions)")
                    self.ax.set_xlabel("Action Index")
                    self.ax.set_ylabel("Angle (°)")
                    self.ax.legend()
                    self.ax.grid(True)
                    self.canvas.draw()
        except Exception as e:
            print("Graph update error:", e)
        finally:
            self.after(1500, self.update_graph)

    # ----------------------------------------------------------------
    # UTILITIES
    # ----------------------------------------------------------------
    def set_ai_indicator(self, is_ai_active: bool):
        """Toggle indicator light color."""
        color = "green" if is_ai_active else "red"
        self.ai_indicator.itemconfig(self.ai_light, fill=color)

    def update_status(self, message):
        self.status_text.set(message)
        self.get_logger().info(message)

    def on_close(self):
        self.update_status("🛑 Shutting down robotDash...")
        if self.ai_process:
            try:
                self.ai_process.terminate()
                self.ai_process.wait(timeout=2)
            except Exception:
                pass
        self.after(500, self.destroy)


# -------------------------------------------------------------------
# ENTRY POINT
# -------------------------------------------------------------------
def main(args=None):
    app = robotDash()
    app.mainloop()
    sys.exit(0)


if __name__ == "__main__":
    main()
