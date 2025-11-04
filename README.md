#  AI Robotic Arm (ROS 2 Humble)

A 2D AI-controlled robotic arm simulator built using Python and ROS 2.  
The arm can respond to keyboard and natural-language commands.

### Build Instructions
```bash
cd ~/ROS2_WS
colcon build --symlink-install
source install/setup.bash
ros2 run ai_robotic_arm arm_simulator
