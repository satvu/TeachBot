#!/usr/bin/env python

import rospy
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE_WRIST = [0, -3.14, 0, -3.14, -1.57, -1]
ROTATE_WRIST_BACK = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class PositionClient:

    def __init__(self):

        self.pub_command = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=1)
        rospy.Subscriber('joint_states', JointState, self.send_command)


    def send_command(self, joints):
        print 'press enter'
        pause = raw_input()
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES
        jointPositions_msg = JointTrajectoryPoint()
        jointPositions_msg.positions = [0, -1.57, 0, -1.57, 0, 0]
        self.pub_command.publish(traj_msg)


if __name__ == '__main__':
    rospy.init_node('request')
    pc = PositionClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




