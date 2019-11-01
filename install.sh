#!/bin/bash

# curl
sudo apt-get update
sudo apt-get -y install curl

# apt dependencies
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install -y ros-kinetic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential

# ROS setup
sudo rosdep init
sudo rosdep fix-permissions
rosdep update
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# apt-get dependencies
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get update
sudo apt-get install -y git-core python-argparse python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers ros-kinetic-xacro ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-cv-bridge ros-kinetic-actionlib ros-kinetic-actionlib-msgs ros-kinetic-dynamic-reconfigure ros-kinetic-trajectory-msgs ros-kinetic-rospy-message-converter ros-kinetic-rosbridge-suite nodejs cmake

# pip dependencies
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py --user
rm get-pip.py
pip install --upgrade --user gTTS gTTS-token pyttsx3 pygame apriltag

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
sudo npm install .

cd ../robot
echo "To finish the installation, please refer to the README.md"
