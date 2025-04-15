#!/bin/zsh

source ./install/setup.sh

# ros2 topic pub -r 10 spin_flag spin_flag_interfaces/msg/SpinFlag "{
#     spin_flag: true
# }"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{
  linear: {x: 0.15, y: 0.15, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}
}"