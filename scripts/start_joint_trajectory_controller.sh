#!/bin/bash
# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

export ROS_MASTER_URI=http://192.168.0.1:11311

export ROS_HOSTNAME=192.168.0.1

source ~/Projects/catkin_arduino_robot_ws/devel/setup.bash

rosservice call /controller_manager/switch_controller "{start_controllers: ['joint_trajectory_controller'], stop_controllers: [], strictness: 2}"

