# clone repo
$ git clone https://github.com/Indraputrabh/resilient_multi_robot_ros2
$ cd multinet-bot-ros2
 
# build
$ colcon build --packages-select multi_net_bot
$ source install/setup.bash
 
# launch core nodes
$ ros2 run multi_net_bot role_manager
$ ros2 run multi_net_bot swarm_manager
$ ros2 run multi_net_bot robot_node  # repeat per robot