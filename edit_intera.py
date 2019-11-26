# Extract contents of intera.sh
f = open("robot/src/intera_sdk/intera.sh","r")
intera = f.read()
f.close()

# Perform Substitutions
intera = intera.replace("robot_hostname=\"robot_hostname.local\"", "robot_hostname=\"jerry.local\"", 1)
intera = intera.replace("your_ip=\"192.168.XXX.XXX\"", "", 1)
intera = intera.replace("#your_hostname=\"my_computer.local\"", "your_hostname=\"$(uname -n).local\"")
intera = intera.replace("ros_version=\"indigo\"","ros_version=\"kinetic\"", 1)

# Write New File
f = open("robot/intera.sh","w")
f.write(intera)
f.close()

# Repeat for startup scripts
intera = intera.replace("ros_version=\"kinetic\"", "ros_version=\"kinetic\"\n\nxrandr --output HDMI-2 --brightness 1 --mode 1366x768 --same-as eDP-1 --output eDP-1 --mode 1366x768\nfirefox https://localhost:8000 &\ncd ~/TeachBot/robot/", 1)
intera = intera.replace("\nEOF\n","\npython start.py\n\nEOF\n")
f = open("robot/start.sh","w")
f.write(intera)
f.close()