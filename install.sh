#!/bin/bash

# apt dependencies
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# apt-get dependencies
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get update
sudo apt-get install -y git-core python-argparse python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter ros-melodic-rosbridge-suite nodejs

# pip dependencies
pip install --upgrade --user gTTS gTTS-token pexpect playsound pyttsx3 pygame

# Merge intera resources
cd robot/src
wstool init .
git clone https://github.com/RethinkRobotics/sawyer_robot.git
wstool merge sawyer_robot/sawyer_robot.rosinstall
wstool update
cd ..
source ~/.bashrc
catkin_make
cp src/intera_sdk/intera.sh intera.sh

# NPM Installs
cd ../browser
npm install .

cd ../robot
echo "To finish the installation, please refer to the README.md"
