# 🤖 DvaitAI – Robotic Arm Simulator (ROS 2 Humble)

A **2-link robotic arm simulator** powered by **Python**, **ROS 2 (Humble)**, **Tkinter**, and **Pygame** — built as the foundation for the **DvaitAI** initiative:  
a journey toward practical, open, and evolving AI + Robotics systems.

---

## 🧭 Journey Summary (Day 1 → Day 7)

| Day | Focus | Description | Link |
|-----|--------|--------------|------|
| **Day 1** | Vision & Foundation | Defined the DvaitAI goal, created GitHub + LinkedIn, and set project roadmap. | [View](docs/day1.md) |
| **Day 2** | ROS 2 Setup | Installed ROS 2 Humble, configured workspace, and tested pub/sub nodes. | [View](docs/day2.md) |
| **Day 3** | Package Creation | Created `ai_robotic_arm` package and simulator base structure. | [View](docs/day3.md) |
| **Day 4** | Real-Time Control | Integrated `/arm_command` & `/arm_status` topics with visualization. | [View](docs/day4.md) |
| **Day 5** | Brand Identity | Launched DvaitAI branding on GitHub, LinkedIn, and YouTube. | [View](docs/day5.md) |
| **Day 6** | Dashboard Build | Created Tkinter-based real-time control dashboard integrated with ROS 2. | [View](docs/day6.md) |
| **Day 7** | Visualization + Release | Added live telemetry visualization, autopilot mode, and released v1.0 demo. | [View](docs/day7.md) |

---

## ✨ Current Features (v1.0)
- ✅ Real-time **ROS 2 control** via `/arm_command`
- ✅ Live **Pygame-based arm visualization**
- ✅ **Tkinter Dashboard** for manual and autopilot modes
- ✅ Publishes live status via `/arm_status`
- ✅ **Autopilot Mode** with random/patterned movement
- ✅ **Live Telemetry Graph** (Matplotlib)
- ✅ **CSV Logging** for data analysis
- ✅ **Pause/Resume Graph** toggle for inspection

---

## 🎥 Demo Video
📹 [Watch DvaitAI v1.0 Demo – ROS 2 Robotic Arm Dashboard](https://youtu.be/Hue3Q42pfck)

See how ROS 2 topics, dashboard control, and telemetry visualization come together —  
a complete system from **manual control to automated motion**.

---

## 🧩 System Architecture
```
    ROS2 Workspace (ros2_ws)
    └── ai_robotic_arm
    ├── ai_robotic_arm/
    │ ├── ai_controller.py
    │ ├── ai_autopilot.py
    │ ├── arm_simulator.py
    │ ├── arm_controller_node.py
    │ ├── robot_dashboard.py
    │ └── arm_log.csv
    ├── package.xml
    ├── setup.py
    └── README.md
```

---

## ⚙️ Tech Stack
| Component | Description |
|------------|-------------|
| **Language** | Python 3.10+ |
| **Framework** | ROS 2 (Humble) |
| **UI** | Tkinter |
| **Simulation** | Pygame |
| **Visualization** | Matplotlib |
| **Logging** | CSV |
| **Control Modes** | Manual / Autopilot |

---

## 🧠 Learnings
Before intelligence comes control.  
This version focuses on mastering **feedback loops**, **data flow**, and **system response** —  
the essentials every AI system needs before learning can begin.

> “A system that can respond with clarity is already one step closer to thinking.”

---

## 🧭 Next Steps – DvaitAI v2
The next version will evolve this simulator into a more autonomous and intelligent system:

- 🔹 Add **rule-based autopilot logic**
- 🔹 Implement **goal-based arm positioning**
- 🔹 Integrate **PID control** for smooth motion
- 🔹 Expand **telemetry** with time and velocity
- 🔹 Prepare for **learning-based extensions**

---

## 💡 Project Vision
**DvaitAI** explores the *duality of intelligence* —  
where creation and control coexist.  
Every system can build or break; the difference lies in how we guide it.

This initiative aims to make robotics **accessible, ethical, and scalable**,  
bridging the gap between academic AI and practical systems.

---

## 🌐 Official Links
- 🔗 **GitHub:** [DvaitAI-tech](https://github.com/DvaitAI-tech)
- 🎥 **YouTube:** [@DvaitAITech](https://www.youtube.com/@DvaitAITech)
- 💼 **LinkedIn:** [Nripender Kumar](https://www.linkedin.com/in/nripender-kumar-200ab81a1/)
- 📩 **Email:** nk.dvaitai@gmail.com

---

## 💬 Motivation
> “Innovation isn’t about perfection — it’s about iteration.  
> Each working system is a foundation for something greater.”

---

## 📜 License
MIT License — Free to use, modify & share.

---

## 🌟 Support the Journey
If you find this project inspiring, **⭐ Star the repo**,  
share it, and subscribe on YouTube for the next phase of **DvaitAI**.

> *DvaitAI — Where Intelligence Meets Duality.*

## ✅ Highlights of this final version

100% accurate (no false AI claims)

Includes truthful tech summary (manual + autopilot only)

Has learning + motivational insight for readers

Clean v1.0 release polish

Sets clear expectations for v2