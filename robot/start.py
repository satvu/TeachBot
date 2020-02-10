#! /usr/bin/env python

import pexpect

# Wait for Firefox to open
ffOpen = False
while not ffOpen:
	child = pexpect.spawn('xdotool search --name "Firefox"')
	ffOpen = child.expect([r'\d+', pexpect.EOF])==0

# Fullscreen Firefox
child = pexpect.spawn('xdotool search --sync --onlyvisible --class "Firefox" windowactivate key F11')
child.expect(pexpect.EOF)

# Roslaunch websocket
p_roslaunch = pexpect.spawn('roslaunch ../browser/websocket.launch')
p_roslaunch.expect('Rosbridge WebSocket server started on port 9090')

# Start node server
p_node = pexpect.spawn('node ../browser/www.js')
p_node.expect('Connected to master at ')

# Rosrun teachbot.py
p_rosrun = pexpect.spawn('rosrun sawyer teachbot.py')
p_rosrun.expect('Ready')

# Refresh Firefox
child = pexpect.spawn('xdotool search --sync --onlyvisible --class "Firefox" windowactivate key F5')
child.expect(pexpect.EOF)

# Wait for user to stop
ctrl = raw_input('Press enter to finish.')