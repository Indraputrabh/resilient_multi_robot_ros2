# How to Build the Project
cd ~/RESILIENT_MULTI_ROBOT_ROS2
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 run multi_net_bot talker

# Source ROS 2 Jazzy base environment
source /opt/ros/jazzy/setup.bash

# Then source your workspace overlay
source ~/RESILIENT_MULTI_ROBOT_ROS2/install/setup.bash
