# TeachBot

## First Setup Only
The following is adapted from http://sdk.rethinkrobotics.com/intera/Workstation_Setup and https://wiki.ros.org/kinetic/Installation/Ubuntu for Ubuntu 16.04 and ROS Kinetic.

### Clone Development Workspace
If you change the directory, make sure you adjust all following commands.
```
$ cd
$ git clone https://github.com/Darbeloff/TeachBot.git
```

### Install Curl and Apt Dependencies
Run the following commands in terminal:
```
$ sudo apt-get update
$ sudo apt-get -y install curl
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
$ sudo apt update
$ sudo apt install -y ros-kinetic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Initialize ROS and Update .bashrc
If you have already initialized ROS, you do not need to run these commands. If you do, you will see warning messages.
```
$ sudo rosdep init
$ sudo rosdep fix-permissions
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Install Apt-Get Dependencies
```
$ curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
$ sudo apt-get update
$ sudo apt-get install -y git-core python-argparse python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers ros-kinetic-xacro ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-cv-bridge ros-kinetic-actionlib ros-kinetic-actionlib-msgs ros-kinetic-dynamic-reconfigure ros-kinetic-trajectory-msgs ros-kinetic-rospy-message-converter ros-kinetic-rosbridge-suite nodejs cmake
```

### Install Pip Dependencies
```
$ curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
$ python get-pip.py --user
$ rm get-pip.py
$ pip install --upgrade --user gTTS gTTS-token pygame apriltag
```

### Merge Intera Resources
```
$ cd robot/src
$ wstool init .
$ git clone https://github.com/RethinkRobotics/sawyer_robot.git
$ wstool merge sawyer_robot/sawyer_robot.rosinstall
$ wstool update
$ cd ..
$ source ~/.bashrc
$ catkin_make
```

### NPM Installs
```
$ cd ../browser
$ sudo npm install .
```

### Set Up Networking
Connect to the robot via the ethernet port on the outside of the Controller.
![Source: http://sdk.rethinkrobotics.com/intera/Workstation_Setup](http://sdk.rethinkrobotics.com/intera/a/images/e/ec/Ethernet_Port.png)

Open Ubuntu's "Settings" tool, and navigate to the "Network" tab. Then, click the gear icon in the "Wired" section.
![Click the gear icon in Network Settings](/images/network_settings.png?raw=true)

Navigate to the IPv4 tab and select "Link-Local Only." Then, click "Apply."
![Enable Link-Local Only](/images/wired_settings.png?raw=true)

You will not be able to use that ethernet port to access the internet until you revert the Method to "Automatic (DHCP)." You may now close the Settings window.

### Set Up Intera
Copy the `intera.sh` file into your ros workspace
```
$ cp ~/TeachBot/robot/src/intera_sdk/intera.sh ~/TeachBot/robot
```

Now you'll need to edit three lines of `intera.sh` specific to your system.
```
$ gedit intera.sh
```

#### Edit `robot_hostname`
Edit the `robot_hostname` field on line 22 to be the name of your robot. For example:
```
robot_hostname="jerry.local"
```
where `jerry` is the name of your robot. This needs to match the name you gave it when you set it up.

#### Replace `your_ip` with `your_hostname`
Comment out the `your_ip` field on line 26 and edit the `your_hostname` field on line 27 as follows:
```
#your_ip="192.168.XXX.XXX"
your_hostname="$(uname -n).local"
```

#### Edit `ros_version`
Edit the `ros_version` field on line 30 to be the version of ROS you are running. Assuming you followed this README, that is:
```
ros_version="kinetic"
```

Save and close `intera.sh` script.

### Verify Environment
```
$ cd ~/TeachBot/robot
$ ./intera.sh
```

You should now be in a new shell titled `intera - http://tom.local:11311` where `tom` is the name of your robot. A useful command for viewing and validating your ROS environment setup is:
```
$ env | grep ROS
```

The important fields at this point:
`ROS_MASTER_URI` - This should now contain your robot's hostname.
`ROS_HOSTNAME` - This should contain your PC's hostname.
Try to get rostopic list from robot by typing following command:
```
$ rostopic list
```

You should see a the rostopic list from the command line output similar as following:
```
/array_topic
/audio_duration
/calibration_command/cancel
/calibration_command/feedback
/calibration_command/goal
/calibration_command/result
/calibration_command/status
/camera_field_calibration/cancel
/camera_field_calibration/feedback
/camera_field_calibration/goal
/camera_field_calibration/result
/camera_field_calibration/status
/cmd2browser
/cmd2shell
/collision/right/collision_detection
/collision/right/debug
/dev_topic
/diagnostics
/engine/task_state
/errors
/head_navigator_button
/intera/endpoint_ids
/io/comms/command
/io/comms/config
/io/comms/io/command
/io/comms/io/config
/io/comms/io/state
/io/comms/state
/io/end_effector/command
/io/end_effector/config
/io/end_effector/state
/io/internal_camera/command
...
```

## To Launch
Navigate to the `TeachBot/robot` directory.
```
$ cd ~/TeachBot/robot
```

Initialize SDK environment
```
$ ./intera.sh
```

Launch the websocket and start Node.js
```
$ roslaunch ../browser/websocket.launch & node ../browser/www.js &
```

Run the TeachBot ROS Node
```
$ rosrun sawyer teachbot.py
```

## Load Modules in Firefox
Open Firefox and go to the url https://localhost:8000

### If Firefox alerts you that your connection is not secure
Click "Advanced"

![Click "Advanced"](/images/insecure_connection.png?raw=true)

Click "Add Exception"

![Click "Add Exception"](/images/add_exception.png?raw=true)

Click "Confirm Security Exception"

![Click "Confirm Security Exception"](/images/confirm_exception.png?raw=true)

At this point, the module should begin, but there should be a pop-up window that reads "Error connecting to websocket server" like this:

![Navigate to localhost:9090 and repeat the security exception process.](/images/connection_error.png?raw=true)

Then, go to the url https://localhost:9090 and repeat the security exception process. After, https://localhost:9090 should simply read `Can "Upgrade" only to "WebSocket".` Navigate back to https://localhost:8000.
### EndIf

You have successfully launched the teaching module!

## Repository Overview
```
.
|
+-- browser                             Files related to the HTML/CSS/JS running in-browser.
|   +-- public                          Front-End Browser Content.
|       +-- audio                       TeachBot speech audio files.
|           +-- module#                 Directories containing subdirectories of audio files, indexed by module.
|               +-- <section_name>/     Directories containing audio files indexed by line number.
|           +-- make_speech.py          Script to generate speech audio files from text scripts.
|       +-- css/                        Style sheets for the browser HTML
|       +-- html/                       Web pages to display in browser.
|       +-- images/                     Images displayed in browser by TeachBot.
|       +-- js                          JavaScript module control scripts and utilities.
|           +-- json/                   JavaScript Object Notation (JSON) files containing the instructions for each module.
|           +-- utils/                  Utility functions for browser.
|           +-- Module.js               Module object class definition.
|           +-- #.js                    Module control scripts for browser indexed by module.
|       +-- text                        The text of what TeachBot says.
|           +-- module#                 Directories containing .txt files of what TeachBot says, indexed by module.
|               +-- <section_name>.txt  Text file containing all lines spoken by TeachBot in a given section.
|       +-- videos/                     Videos played in browser by TeachBot.
|   +-- routes/                         Node.js routes used by app.js.
|   +-- views/                          App templates.
|   +-- app.js                          Main configuration file for TeachBot browser.
|   +-- CMakeLists.txt                  Node CMakeLists.
|   +-- package-lock.json               NPM install utility.
|   +-- package.json                    NPM install utility.
|   +-- package.xml                     Node package manifest.
|   +-- websocket.launch                ROS launch file for web module.
|   +-- www.js                          Starts Node.js server.
|
+-- images/                             Images for the README.
|
+-- robot                               Python ROS library responsible for communicating with the robot.
|   +-- src                             Source code.
|       +-- sawyer                      TeachBot ROS package for the RethinkRobotics Sawyer.
|           +-- Learner_Responses/      Data collected from user subject tests.
|           +-- msg/                    Custom message files for information transfer to and from the browser.
|           +-- src/                    Python source code.
|               +-- teachbot.py         ROS listener controlling the robot and receiving commands from the browser.
|               +-- <other>.py          Utility classes and functions used by module control scripts.
|           +-- CMakeLists.txt          ROS package CMakeLists.
|           +-- package.xml             ROS package manifest.
|           +-- setup.py                ROS package setup script.
|   +-- sslcert/                        SSL certificate files required for setting up HTTPS connection.
|   +-- logo.png                        TeachBot logo to be displayed on Sawyer head display.
|   +-- safety1.mp3                     Audio file to be played when TeachBot limb exits safety zone.
|   +-- safety2.mp3                     Audio file to be played when TeachBot limb resets to within the safety zone.
```
The project is organized in two segments:
1) JavaScript Node application, responsible for coordinating the module and displaying content in the browser, and
2) Python ROS library, responsible for communicating with the robot.

### JavaScript Node Application
The JavaScript scripts controlling the browser are located [here](/browser/public/js) and are indexed by module (e.g. `1.js`, `2.js`, and so on). The layout is simple:

1. Construct Module object.
2. Initialize main() function, which starts the Module.

This should be the last script included in the HTML.

The crux of these files is `Module.js`, the constructor of the Module object. Upon construction, a Module sets up ROS communication with the robot, prepares the browser for a Module, and loads all of the resources needed to run the module including the text, audio, and JSON files. Once everything is loaded, it runs the `main()` function provided by the calling script. The `main()` function should always conclude with a call to `Module.start()`, which begins recursively iterating through the instructions in the JSON file.

The server running the browser half of the application can be started using the command described above:
```
$ roslaunch ../browser/websocket.launch & node ../browser/www.js &
```

### Python ROS Library
All Python ROS packages are located in [the robot source directory](/robot/src/).

With the exception of `arduino_files` and the files imported from other repositories, each package is named after the robot it commands (e.g. `sawyer` should be used when connected to the RethinkRobotics Sawyer cobot). In the source directory of each package, there should be a `teachbot.py` script that runs the central listener, receiving instructions from the JavaScript Module over ROS, forwarding those commands to the robot over Ethernet, and communicating data from the robot back to the JavaScript Module.

The main `teachbot.py` scripts are run using the command described above:
```
$ # This is just the format of the command. Do not actually enter <name_of_cobot>.
$ rosrun <name_of_cobot> teachbot.py
```

### Block Diagram of Command Flow
![](/images/software_overview.svg?raw=true)
