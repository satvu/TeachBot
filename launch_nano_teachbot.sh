#!/bin/bash

echo "Launching TeachBot..."
cd robot
gnome-terminal --tab -- /bin/bash -c 'roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=169.254.157.0 kinematics_config:=${HOME}/TeachBot/robot/src/ur5e_robot_calibration.yaml'
sleep 2 # Wait until the execution is complete
gnome-terminal --tab -- /bin/bash -c 'roslaunch ../browser/websocket.launch & node ../browser/www.js &'
sleep 2
gnome-terminal --tab -- /bin/bash -c 'rosrun ur teachbot.py'
echo "TeachBot launched."
