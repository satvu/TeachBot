#!/usr/bin/env python
'''
Created on July 1, 2019
Author: Albert Go (albertgo@mit.edu)
'''

import rospy
import sensor_msgs.msg as smsg
from ur.msg import Position, PositionFeedback, PositionResult
from std_msgs.msg import Bool, Int32

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE_WRIST = [0, -3.14, 0, -3.14, -1.57, -1]
ROTATE_WRIST_BACK = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]

JOINT_MOVE = 0
ADMITTANCE = 1

class PositionClient:

    def __init__(self):

        self.progress = []
        self.completed = False

        rospy.Subscriber('position_feedback', PositionFeedback, self.feedback_cb)
        rospy.Subscriber('position_result', PositionResult, self.send_positions)

        self.pub_goal = rospy.Publisher('position', Position, queue_size=1)
        self.pub_mode = rospy.Publisher('ur_mode', Int32, queue_size=1)

    #for GoToJointAngles
    # def send_positions(self, data):
    #     if data.initialized == True and data.completed == True:
    #         target = SCARA
    #         print 'Enter a request, if no specific request, type and enter "none"'
    #         user = raw_input()
    #         print 'Sending Goal'
    #         if user == '0':
    #             target = ZERO 
    #         elif user == '1':
    #             target = ROTATE_WRIST 
    #         elif user == '2':
    #             target = ROTATE_WRIST_BACK

    #         goal = Position(base=target[0], shoulder=target[1], elbow=target[2], wrist1=target[3], wrist2=target[4], wrist3=target[5])
    #         print(goal)
    #         self.pub_goal.publish(goal)
    #         return
    #     elif data.initialized == True and data.completed != True:
    #         print 'Awaiting completion. Progress:', self.progress
    #     else:
    #         print 'Waiting for robot to initialize.'

    #for admittance
    def send_positions(self, data):
        if data.initialized == True and data.completed == True:
            user = raw_input()
            self.pub_mode.publish(JOINT_MOVE)

    def feedback_cb(self, data):
        #print data.progress
        self.progress = data.progress


if __name__ == '__main__':
    rospy.init_node('request')
    pc = PositionClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




