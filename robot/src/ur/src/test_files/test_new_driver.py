#!/usr/bin/env python

import rospy
import sensor_msgs.msg as smsg
from ur.msg import Position, PositionFeedback, PositionResult
from std_msgs.msg import Bool, Int32