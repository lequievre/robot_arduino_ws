#!/bin/bash
# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

export ROS_MASTER_URI=http://192.168.0.1:11311

export ROS_HOSTNAME=192.168.0.1

sudo chmod 777 /dev/ttyACM0

roslaunch rosserial_server serial.launch port:=/dev/ttyACM0 baud:=57600

# 115200
#rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

