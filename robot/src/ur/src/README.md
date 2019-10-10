# TeachBot UR

## Running the UR package
From TeachBot/robot run the following commands (each in its own terminal)

$ roslaunch ../browser/websocket.launch & node ../browser/www.js &

$ rosrun ur ur_rtde_driver.py

$ rosrun ur command_manager.py

$ rosrun ur limb.py

$ rosrun ur teachbot.py


Currently testing using the JSON file "42.json" - go to the link localhost:8000/urTests to run this file's commands. 

## Potential Issues 
If you cannot seem to connect the robot, press pause and then press play again to reconnect the robot. You might have accidentally disconnected when pausing or stopping in the middle of a running module/program. If this does not work still, refresh the "/urTests" link. 

## Testing directly 

If you want to avoid using the browser module, test the ur directly using test_requests.py. 



