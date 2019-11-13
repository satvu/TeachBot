#!/bin/bash

# Merge intera resources
cd robot/src
wstool init .
git clone https://github.com/RethinkRobotics/sawyer_robot.git
wstool merge sawyer_robot/sawyer_robot.rosinstall
wstool update
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make
cd ..
python edit_intera.py
chmod +x robot/intera.sh

# NPM Installs
cd browser
sudo npm install .

cd ..