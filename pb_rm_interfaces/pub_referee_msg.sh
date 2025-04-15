#!/bin/zsh

source ./install/setup.sh

ros2 topic pub -r 1 /referee/game_status pb_rm_interfaces/msg/GameStatus "{
    game_progress: 4,
    stage_remain_time: 300,}" &
ros2 topic pub -r 10 /referee/robot_status pb_rm_interfaces/msg/RobotStatus "{
    current_hp: 500,
    hp_deduction_reason: 0,
    shooter_17mm_1_barrel_heat: 50, 
    projectile_allowance_17mm: 100,
    is_hp_deduced: true, 
    }" 
