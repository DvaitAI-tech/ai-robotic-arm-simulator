# 🤖 AI Robotic Arm – ROS 2 Humble

A **2-link robotic-arm simulator** built with **Python**, **Pygame**, and **ROS 2 (Humble)**.  
It can receive movement commands through ROS 2 topics and publish live arm-status updates — the foundation for an AI-controlled robotic system.  

---

## ✨ Current Features (Day 5)
- ✅ Real-time ROS 2 control using `/arm_command` topic  
- ✅ Live simulator visualization built with Pygame  
- ✅ Publishes feedback on `/arm_status`  
- ✅ Basic AI command parser for natural-language-like inputs  
- ✅ Smooth 2-joint motion with clean shutdown handling  

---

## 🧠 Upcoming Goals (Day 6 →)
- 🤖 Integrate an **AI Agent** that automatically generates movement commands  
- 🧩 Add a **control dashboard / web UI** for visualization  
- 📡 Publish detailed telemetry (angles, velocities, actions)  
- 💡 Launch open-source release and start early monetization experiments  

---

## 🧩 System Architecture
```
ROS2 Workspace (ros2_ws)
└── ai_robotic_arm
    ├── ai_robotic_arm/
    │   ├── ai_controller.py          # Text command parser
    │   ├── arm_simulator.py          # Base visualization
    │   ├── arm_command_subscriber.py # Command listener (earlier version)
    │   ├── arm_status_publisher.py   # Status publisher
    │   └── arm_controller_node.py    # Real-time control + visualization
    ├── package.xml
    ├── setup.py
    └── README.md
```

---

## ⚙️ Installation & Build

```bash
# 1️⃣ Clone the repository
cd ~/ros2_ws/src
git clone git@github.com-work:kukrumku/ai-robotic-arm-simulator.git

# 2️⃣ Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# 3️⃣ Source the setup file
source install/setup.bash
```

---

## ▶️ Run the Simulation

### Start the controller node
```bash
ros2 run ai_robotic_arm arm_controller
```

### Send movement commands
```bash
ros2 topic pub /arm_command std_msgs/String "data: 'move left'"
ros2 topic pub /arm_command std_msgs/String "data: 'move up'"
ros2 topic pub /arm_command std_msgs/String "data: 'pick'"
```

### Observe arm status
```bash
ros2 topic echo /arm_status
```

✅ Watch the robotic arm respond to your commands live in the Pygame window!  

---



## 💡 Project Vision
This is part of a year-long journey to build **AI-driven robotic systems from scratch**,  
with a long-term target to develop a **AI + Robotics product ecosystem** from India 🇮🇳.  

Follow the journey on [LinkedIn](https://www.linkedin.com/in/your-linkedin-handle) for daily progress!  

---

## 🧰 Tech Stack
| Component | Description |
|------------|-------------|
| **Language** | Python 3.10 + |
| **Framework** | ROS 2 Humble (rclpy) |
| **Visualization** | Pygame |
| **Messaging** | `std_msgs/String` topics |
| **AI Integration (soon)** | Local LLM / API Agents |
| **Build** | `colcon build --symlink-install` |

---

## 🤝 Contributing
1. Fork the repo  
2. Create a new feature branch  
3. Commit changes and open a PR  
4. Suggestions and improvements are welcome!  

---

## 📜 License
MIT License — Feel free to use, modify, and build on this project.  

---

## 🌟 Support the Journey
If you find this project interesting, please **⭐ Star** the repository and share it!  
Your support helps this AI + Robotics initiative grow.
