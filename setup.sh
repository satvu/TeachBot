#!/bin/bash

# Check Ubuntu version
ver=0
if [[ $(lsb_release -a | grep "Description:" | grep "Ubuntu 18") ]] ; then
	ver=18
elif [[ $(lsb_release -a | grep "Description:" | grep "Ubuntu 16") ]] ; then
	ver=16
else
	echo "This version of Ubuntu is not supported"
	exit 0
fi

# Merge intera resources
cd robot/src
wstool init .
git clone https://github.com/RethinkRobotics/sawyer_robot.git
wstool merge sawyer_robot/sawyer_robot.rosinstall
wstool update
cd ..
if [ $ver -eq 18 ] ; then
	source /opt/ros/melodic/setup.bash
elif [ $ver -eq 16 ] ; then
	source /opt/ros/kinetic/setup.bash
fi
catkin_make
cd ..
if [ $ver -eq 18 ] ; then
	python edit_intera.py melodic
elif [ $ver -eq 16 ] ; then
	python edit_intera.py kinetic
fi
chmod +x robot/intera.sh
chmod +x robot/start.sh
cp robot/start.sh ~/Desktop/start_TeachBot.sh
chmod +x ~/Desktop/start_TeachBot.sh

# NPM Installs
cd browser
sudo npm install .

cd ..