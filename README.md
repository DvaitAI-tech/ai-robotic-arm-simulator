# 🤖 DvaitAI – AI Robotic Arm (ROS 2 Humble)

A **2-link robotic arm simulator** powered by **Python**, **ROS 2 (Humble)**, and **Pygame** — evolving into a self-learning AI robotic system under the **DvaitAI** initiative.

---

## 🧭 Journey Summary (Day 1 → Day 6)

| Day | Focus | Description | Link |
|-----|--------|--------------|------|
| **Day 1** | Vision & Foundation | Defined the goal, created GitHub + LinkedIn, and set DvaitAI’s direction. | [View](docs/day1.md) |
| **Day 2** | ROS 2 Setup | Installed ROS 2 Humble, configured workspace, and tested pub/sub. | [View](docs/day2.md) |
| **Day 3** | ROS 2 Package Creation | Created `ai_robotic_arm` package and simulator base structure. | [View](docs/day3.md) |
| **Day 4** | Real-Time Control | Integrated `/arm_command` & `/arm_status` topics with visualization. | [View](docs/day4.md) |
| **Day 5** | Brand Identity & Integration | Created YouTube, GitHub updates, and launched DvaitAI identity. | [View](docs/day5.md) |
| **Day 6** | AI Autopilot + Dashboard | Integrated AI Autopilot and built Tkinter-based real-time control dashboard with ROS 2. | [View](./Day6.md) |


---

## ✨ Current Features
- ✅ Real-time ROS 2 control via `/arm_command`  
- ✅ Live Pygame visualization  
- ✅ Publishes live arm status updates  
- ✅ Natural-language-style command parser  
- ✅ AI Autopilot for autonomous motion  
- ✅ Tkinter-based real-time dashboard for manual control  
- ✅ Live status visualization from ROS 2 topics (`/arm_status`)


---

## 🧠 Upcoming Goals
- 🧩 Add **AI Mode Toggle** inside dashboard  
- 🧩 Implement **pattern-learning system** for adaptive motion  
- 📊 Visualize telemetry data (angles, actions, and time series)  

## 🎥 Demo
Watch the live dashboard in action:  
📹 [DvaitAI | Day 6 – Real-Time Robotic Arm Dashboard (Tkinter + ROS 2)](https://youtu.be/2UlGpsEMKyg)


---

## 🧩 System Architecture
```
    ROS2 Workspace (ros2_ws)
    └── ai_robotic_arm
    ├── ai_robotic_arm/
    │ ├── ai_controller.py
    │ ├── arm_simulator.py
    │ ├── arm_controller_node.py
    │ ├── ai_autopilot.py
    ├── package.xml
    ├── setup.py
    └── README.md
```

---

## 💡 Project Vision
**DvaitAI** explores the *duality of intelligence* —  
where creation and control coexist.  
Each algorithm can build or destroy — the outcome depends on how we use it.

---

## 🌐 Official Links
- 🔗 **GitHub:** [DvaitAI-tech](https://github.com/DvaitAI-tech)  
- 🎥 **YouTube:** [@DvaitAITech](https://www.youtube.com/@DvaitAITech)  
- 💼 **LinkedIn:** [Nripender Kumar](https://www.linkedin.com/in/nripender-kumar-200ab81a1/)  
- 📩 **Email:** nk.dvaitai@gmail.com  

---

## 💪 Motivation
> “Innovation is not born in perfection — it’s forged through curiosity, control, and chaos.”  

---

## 📜 License
MIT License — Free to use, modify & share.  

---

## 🌟 Support the Journey
If you like this project, **⭐ Star** the repo, share it, and follow DvaitAI on YouTube for more AI + Robotics progress.  
> *DvaitAI — Where Intelligence Meets Duality.*
