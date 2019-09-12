# TeachBot

## First Setup Only
Visit https://docs.google.com/document/d/17coZ1wjAkE_KJrHG-zW-HlLgybr0TfxPnbxPHMxYEns/edit?usp=sharing to set up your Sawyer and Workstation.

Make sure you have the following additional packages installed:
```
$ sudo apt-get install ros-kinetic-rosbridge-suite
$ curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
$ sudo apt-get install -y nodejs
$ sudo apt install python-pip
$ pip install --upgrade pip
$ pip install gTTS
$ pip install gTTS --upgrade
$ pip install gTTS-token --upgrade --user
$ sudo easy_install pexpect
```

After cloning, navigate to the `teachbot_ws` folder
```
$ cd teachbot/teachbot_ws
```

Compile the catkin/ROS workspace
```
$ source /opt/ros/kinetic/setup.bash
$ catkin_make
```

Copy over the necessary files from the intera SDK. You may need to change the directory if you did not follow the ReadMe on the Google Drive.
```
$ cp -r ~/ros_ws/src/intera_common src/intera_common
$ cp -r ~/ros_ws/src/intera_sdk src/intera_sdk
$ cp -r ~/ros_ws/src/sawyer_robot src/sawyer_robot
$ catkin_make
```

## To Launch
Make sure you are in the `teachbot/teachbot_ws` directory. 

Initialize SDK environment
```
$ ./intera.sh
```

Launch the websocket and start Node.js
```
$ roslaunch websocket.launch & node src/teachbot_express/bin/www.js &
```

Run the TeachBot ROS Node
```
$ rosrun teachbot_ros module1.py
```
replacing `module1.py` with the desired module.

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
+-- images/
|
+-- teachbot_ws
|   +-- build/
|   +-- devel/
|   +-- src                               Source code.
|       +-- arduino_files/                Scripts for running peripheral devices on Arduino.
|       +-- intera_common/                See [Intera Common](https://github.com/RethinkRobotics/intera_common).
|       +-- intera_sdk/                   See [Intera SDK](https://github.com/RethinkRobotics/intera_sdk).
|       +-- sawyer_robot/                 See [Sawyer PyKDL Library](https://github.com/rupumped/sawyer_pykdl).
|       +-- teachbot_express              Browser utilities.
|           +-- bin/
|           +-- node_modules/
|           +-- public                    Resources used by the client browser.
|               +-- audio                 TeachBot speech audio files.
|                   +-- module#/          Directories containing audio files of TeachBot speech indexed by module.
|                   +-- make_speech.py    Script to generate speech audio files from text scripts.
|               +-- css/                  Style sheets for the browser HTML
|               +-- html/                 Web pages to display in browser.
|               +-- images/               Images displayed in browser by TeachBot.
|               +-- js                    JavaScript module control scripts and utilities.
|                   +-- utils/            Utility functions for browser.
|                   +-- #.js              Module control scripts for browser indexed by module.
|               +-- text/                 What TeachBot says.
|               +-- videos/               Videos played in browser by TeachBot.
|           +-- routes/                   Node.js routes used by app.js.
|           +-- sslcert/
|           +-- views/
|           +-- app.js                    Starts Node.js server.
|       +-- teachbot_ros                  Python ROS library responsible for communicating with the robot.
|           +-- cfg/
|           +-- Learner_Responses/        Data collected from user subject tests.
|           +-- src/                      Python source code.
|               +-- module#.py            Module control scripts for robot indexed by module.
|               +-- <other>.py            Utility classes and functions used by module control scripts.
|   +-- sslcert/
```
The project is organized in two segments:
1) JavaScript Node application, responsible for displaying content in the browser, and
2) Python ROS library, responsible for communicating with the robot.

### JavaScript Node Application
The JavaScript scripts controlling the browser are located [here](/teachbot_ws/src/teachbot_express/public/js) and are indexed by module. The crux of these files is the `init()` function that initiates ROS communication between the browser and the Python shell and subscribes to the `cmd2browser` topic to receive commands from the shell. These messages are of type `Int32` and consist of the index of the command to run. The subscriber function is essentially a massive switch statement commanding the browser to display graphics and play audio depending on the index received on the topic. After performing the prescribed sequence, the script increments the index and passes it to the Python shell by publishing it on  the `cmd2shell` topic.

### Python ROS Library
All Python scripts are located in [the teachbot_ros source directory](/teachbot_ws/src/teachbot_ros/src).

The main files have names beginning in 'module' (e.g. `module1.py`). These files are run using the command described above:
```
$ rosrun teachbot_ros module1.py
```
The scripts are structured according to `module_template.py`. Their main function is as a ROS listener awaiting instructions from the browser and monitoring the robot ROSTopics. Each file contains the function `rx_command(self, data)`, which receives instructions from the browser in the form of ROS messages on the `cmd2shell` topic. Complementing the JavaScript `init()` subscriber, `rx_command()` also acts as essentially a massive switch statement. Whenever an index is received from the browser on the `cmd2shell` topic, `rx_command` performs a sequence of tasks corresponding to the index received, then publishes the index to the `cmd2browser` topic to inform the browser the sequence is complete.

## Editing the Script
The TeachBot script for each module can be found in [the text directory](/teachbot_ws/src/teachbot_express/public/text). Each line is synthesized into its own audio file and played by a single call to the play() method in JavaScript. There can be no empty lines in the script file.

After editing the script, navigate to [the audio directory](/teachbot_ws/src/teachbot_express/public/audio) and run the following command, replacing both instances of `module1` with the module for which you wish to generate audio:
```
$ python make_speech.py ../module_text/module1.txt module1/
```
The `make_speech.py` script uses Google Text-to-Speech to synthesize audio files from each line of the input txt file and saves them in the given directory. The corresponding JavaScript module script downloads and plays these files.

Additionally, you may wish to update the JavaScript file with the new text. To do so, navigate to [the js directory](/teachbot_ws/src/teachbot_express/public/js) and run the following command, replacing `1.js` with the name of the JS file you wish to update:
```
$ python js_update.py 1.js
```
The script will generate a new .js file, `updated_1.js`. Review this file and copy its contents into `1.js`.
