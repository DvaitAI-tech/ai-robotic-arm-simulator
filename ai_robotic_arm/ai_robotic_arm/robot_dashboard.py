#!/usr/bin/env python3
"""
🤖 robotDash - DvaitAI Robotic Arm Dashboard (ROS 2 + Tkinter)
Publishes commands to /arm_command, listens to /arm_status, and launches AI autopilot.
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


class robotDash(tk.Tk, Node):
    """Combined ROS2 Node + Tkinter Dashboard."""

    def __init__(self):
        # Initialize ROS
        rclpy.init(args=None)
        Node.__init__(self, 'robot_dash_node')

        # Initialize GUI
        tk.Tk.__init__(self)
        self.title("🤖 robotDash - DvaitAI Robotic Arm Dashboard")
        self.geometry("600x600")
        self.resizable(False, False)

        # ROS Publisher & Subscriber
        self.cmd_pub = self.create_publisher(String, '/arm_command', 10)
        self.ai_stop_pub = self.create_publisher(String, '/ai_stop', 10)

        self.create_subscription(String, '/arm_status', self.arm_status_callback, 10)

        # GUI Variables
        self.status_text = tk.StringVar(value="ROS Node Ready ✅")
        self.mode_var = tk.StringVar(value="Manual")
        self.arm_feedback = tk.StringVar(value="No updates yet.")

        # AI Process (autopilot)
        self.ai_process = None

        # Build Interface
        self.create_widgets()

        # ROS spinning in background
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Graceful shutdown
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ----------------------------------------------------------------
    # ROS SPIN THREAD
    # ----------------------------------------------------------------
    def spin_ros(self):
        """Spins the ROS2 node to receive messages asynchronously."""
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

        # --- Controls ---
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

        ttk.Button(control_frame, text="[ Pick ]", command=lambda: self.send_command("pick")).grid(
            row=4, column=0, sticky='ew', pady=5, padx=(0, 5))
        ttk.Button(control_frame, text="[ Drop ]", command=lambda: self.send_command("drop")).grid(
            row=4, column=1, sticky='ew', pady=5, padx=(5, 0))

        # --- Mode & Status ---
        status_frame = ttk.LabelFrame(main_frame, text="Mode & Status", padding="10")
        status_frame.grid(row=1, column=0, sticky='ew', pady=10)

        ttk.Label(status_frame, text="Mode:").grid(row=0, column=0, padx=5, pady=5, sticky='w')

        ttk.Radiobutton(
            status_frame,
            text="Manual",
            variable=self.mode_var,
            value="Manual",
            command=self.switch_to_manual_mode
        ).grid(row=0, column=1, padx=5)

        ttk.Radiobutton(
            status_frame,
            text="AI",
            variable=self.mode_var,
            value="AI",
            command=self.switch_to_ai_mode
        ).grid(row=0, column=2, padx=5)

        ttk.Label(status_frame, text="Current Status:").grid(row=1, column=0, sticky='w', padx=5)
        ttk.Label(status_frame, textvariable=self.status_text,
                  font=('Helvetica', 10, 'bold'),
                  anchor='w').grid(row=1, column=1, columnspan=2, sticky='ew', padx=5)

        # --- Arm Feedback ---
        feedback_frame = ttk.LabelFrame(main_frame, text="Arm Feedback (/arm_status)", padding="10")
        feedback_frame.grid(row=2, column=0, sticky='ew', pady=10)
        ttk.Label(feedback_frame, textvariable=self.arm_feedback,
                  font=('Courier', 11, 'bold'),
                  anchor='center').pack(fill='x')

        # Exit
        ttk.Button(main_frame, text="[ Exit ]", command=self.on_close).grid(
            row=3, column=0, sticky='ew', pady=10)

        # Responsive Layout
        main_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)
        status_frame.grid_columnconfigure(1, weight=1)
        status_frame.grid_columnconfigure(2, weight=1)

    # ----------------------------------------------------------------
    # ROS COMMUNICATION
    # ----------------------------------------------------------------
    def send_command(self, key: str):
        """Publishes mapped command to /arm_command."""
        if self.mode_var.get() == "AI":
            self.update_status("❌ Cannot send manual commands in AI mode.")
            return

        msg = String()
        msg.data = key
        self.cmd_pub.publish(msg)
        self.update_status(f"✅ Sent → {msg.data}")

    def arm_status_callback(self, msg):
        """Handles incoming messages from /arm_status."""
        feedback = msg.data
        self.after(0, lambda: self.arm_feedback.set(f"📡 {feedback}"))

    # ----------------------------------------------------------------
    # MODE SWITCHING
    # ----------------------------------------------------------------
    def switch_to_ai_mode(self):
        """Starts AI autopilot subprocess."""
        self.update_status("🧠 Switching to AI Mode... Starting autopilot...")
        if self.ai_process is None:
            try:
                # Start ROS2 AI autopilot
                self.ai_process = subprocess.Popen(
                    ["ros2", "run", "ai_robotic_arm", "ai_autopilot"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                )
                self.update_status("🤖 AI autopilot running...")
            except Exception as e:
                self.update_status(f"❌ Failed to start AI autopilot: {e}")
                self.mode_var.set("Manual")

    def switch_to_manual_mode(self):
        """Stops AI autopilot subprocess."""
        self.update_status("🧩 Switching to Manual Mode...")

        # Send stop command to AI autopilot node
        stop_msg = String()
        stop_msg.data = "stop"
        self.ai_stop_pub.publish(stop_msg)
        self.update_status("🛑 Sent stop signal to AI autopilot...")

        # Kill subprocess if still alive
        if self.ai_process:
            try:
                self.ai_process.terminate()
                self.ai_process.wait(timeout=2)
            except Exception as e:
                self.update_status(f"⚠️ Error stopping AI autopilot: {e}")
            self.ai_process = None

        self.update_status("✅ Manual mode active.")


    # ----------------------------------------------------------------
    # UTILITIES
    # ----------------------------------------------------------------
    def update_status(self, message):
        """Updates GUI and logs message."""
        self.status_text.set(message)
        self.get_logger().info(message)

    def on_close(self):
        """Clean exit for GUI and ROS."""
        self.update_status("🛑 Shutting down robotDash...")

        # Stop AI process if running
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
