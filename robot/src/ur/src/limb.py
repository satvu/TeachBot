# Imports from TeachBot
from std_msgs.msg import String
from geometry_msgs.msg import (
	PoseStamped,
	Pose
)

# Intera (Imports from TeachBot)
import intera_interface
from intera_interface import (
	Limb,
	Lights,
	CHECK_VERSION
)
from intera_motion_msgs.msg import (
	TrajectoryOptions
)
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (
	MotionTrajectory,
	MotionWaypoint,
	MotionWaypointOptions,
	InteractionOptions
)
from intera_motion_interface.utility_functions import (int2bool, bool2int, boolToggle)
from intera_core_msgs.srv import (
	SolvePositionFK,
	SolvePositionFKRequest
)
from sensor_msgs.msg import JointState

# ur_rtde imports
import rospy
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool

# Position CONSTANTS
SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]

class Limb:
    def __init__(self):
        self.initialized = False
        self.completed = False

        self.current_pos = [0,0,0,0,0,0]
        self.req = []
        self.command_mode = 0
        self.success = [0,0,0,0,0,0]

        ########################
        #####BASE VARIABLES#####
        ########################
        self.base_error_prev = 0
        self.base_final = 0
        self.base_success = False

        ########################
        ###SHOULDER VARIABLES###
        ########################
        self.shoulder_error_prev = 0
        self.shoulder_final = 0
        self.shoulder_success = False

        ########################
        ####ELBOW VARIABLES#####
        ########################
        self.elbow_error_prev = 0
        self.elbow_final = 0
        self.elbow_success = False

        ########################
        ####WRIST1 VARIABLES####
        ########################
        self.wrist1_error_prev = 0
        self.wrist1_final = 0
        self.wrist1_success = False

        ########################
        ####WRIST2 VARIABLES####
        ########################
        self.wrist2_error_prev = 0
        self.wrist2_final = 0
        self.wrist2_success = False

        ########################
        ####WRIST3 VARIABLES####
        ########################
        self.wrist3_error_prev = 0
        self.wrist3_final = 0
        self.wrist3_success = False

        # Subscriptions
		rospy.Subscriber('position', Position, self.command)
		rospy.Subscriber('joint_states', JointState, self.cb_joint_states)

        # Publishes to
		self.pub_setpoint   = rospy.Publisher('setpoint', Setpoint, queue_size= 1)
		self.pub_posfeedback = rospy.Publisher('position_feedback', PositionFeedback, queue_size=1)
		self.pub_posresult = rospy.Publisher('position_result', PositionResult, queue_size=1)

		rospy.Timer(rospy.Duration(0.0008), self.cb_publish)    

	def cb_joint_states(self, data):

		self.current_pos  = data.position

		if self.initialized == False:
			print 'Initializing...'
			self.req = [0, -3.14, 0, -3.14, -1.57, 0, 'none']
		elif len(self.req) > 0: 
			print 'Completing request'

		try:

			if self.req[6] == 'scara':
				print 'scara'
				self.command_mode = 1
			elif self.req[6] == 'zero':
				self.command_mode = 2

			base = self.current_pos[0]
			base_req = self.req[0]
			self.move_base(base, base_req)

			shoulder = self.current_pos[1]
			shoulder_req = self.req[1]
			self.move_shoulder(shoulder, shoulder_req)

			elbow = self.current_pos[2]
			elbow_req = self.req[2]
			self.move_elbow(elbow, elbow_req)

			wrist1 = self.current_pos[3]
			wrist1_req = self.req[3]
			self.move_wrist1(wrist1, wrist1_req)

			wrist2 = self.current_pos[4]
			wrist2_req = self.req[4]
			self.move_wrist2(wrist2, wrist2_req)

			wrist3 = self.current_pos[5]
			wrist3_req = self.req[5]
			self.move_wrist3(wrist3, wrist3_req)

		except:
			pass

		if sum(self.success) == 6:
			self.command_mode = 0
			self.completed = True
			result = PositionResult()

			if self.initialized == False:
				print 'Initialized, ready to comply'
				self.initialized = True
				result.initialized = self.initialized
				result.completed = self.completed
				self.pub_posresult.publish(result)
				self.success = [0,0,0,0,0,0]
				self.req = []
			else:
				print 'Task Completed'
				result.final = self.current_pos
				result.completed = self.completed
				result.initialized = self.initialized
				self.pub_posresult.publish(result)
				self.success = [0,0,0,0,0,0]
				self.req = []
		else:

			self.completed = False

			feedback = PositionFeedback()
			feedback.progress = self.current_pos
			self.pub_posfeedback.publish(feedback)

			result = PositionResult()
			result.initialized = self.initialized
			result.completed = self.completed
			self.pub_posresult.publish(result)



	def command(self, req):

		self.completed = False
		self.success = [0,0,0,0,0,0]

		try:
			self.req.append(req.base)
			self.req.append(req.shoulder)
			self.req.append(req.elbow)
			self.req.append(req.wrist1)
			self.req.append(req.wrist2)
			self.req.append(req.wrist3)
			self.req.append(req.head)
		except:

			pass

	def move_base(self, cur_pos, req_pos):

		if self.command_mode == 1:
			req_pos = 0
		if self.command_mode == 2:
			req_pos = 0

		error = req_pos-cur_pos
		derivative = error-self.base_error_prev
		self.base_final = error+derivative

		if self.base_final > 1:
			self.base_final = 1
		elif self.base_final < -1:
			self.base_final = -1
		elif self.base_final > 0 and self.base_final < 0.08:
			self.base_final = 0.08
		elif self.base_final < 0 and self.base_final > -0.08:
			self.base_final = -0.08

		if cur_pos == req_pos:
			self.success[0] = 1

		#print self.base_final

		self.base_error_prev = error

	def move_shoulder(self, cur_pos, req_pos):

		if self.command_mode == 1:
			req_pos = -3.14
		if self.command_mode == 2:
			req_pos = -1.57

		error = req_pos-cur_pos
		derivative = error-self.shoulder_error_prev
		self.shoulder_final = error+derivative

		if self.shoulder_final > 1:
			self.shoulder_final = 1
		elif self.shoulder_final < -1:
			self.shoulder_final = -1
		elif self.shoulder_final > 0 and self.shoulder_final < 0.08:
			self.shoulder_final = 0.08
		elif self.shoulder_final < 0 and self.shoulder_final > -0.08:
			self.shoulder_final = -0.08

		if cur_pos == req_pos:
			self.success[1] = 1

		#print self.shoulder_final

		self.shoulder_error_prev = error

	def move_elbow(self, cur_pos, req_pos):

		if self.command_mode == 1:
			req_pos = 0
		if self.command_mode == 2:
			req_pos = 0 

		error = req_pos-cur_pos
		derivative = error-self.elbow_error_prev
		self.elbow_final = error+derivative

		if self.elbow_final > 1:
			self.elbow_final = 1
		elif self.elbow_final < -1:
			self.elbow_final = -1
		elif self.elbow_final > 0 and self.elbow_final < 0.08:
			self.elbow_final = 0.08
		elif self.elbow_final < 0 and self.elbow_final > -0.08:
			self.elbow_final = -0.08

		if cur_pos == req_pos:
			self.success[2] = 1

		#print self.self.elbow_final

		self.elbow_error_prev = error

	def move_wrist1(self, cur_pos, req_pos):

		if self.command_mode == 1:
			req_pos =  -3.14
		if self.command_mode == 2:
			req_pos = -1.57

		error = req_pos-cur_pos
		derivative = error-self.wrist1_error_prev
		self.wrist1_final = error+derivative

		if self.wrist1_final > 1:
			self.wrist1_final = 1
		elif self.wrist1_final < -1:
			self.wrist1_final = -1
		elif self.wrist1_final > 0 and self.wrist1_final < 0.08:
			self.wrist1_final = 0.08
		elif self.wrist1_final < 0 and self.wrist1_final > -0.08:
			self.wrist1_final = -0.08

		if cur_pos == req_pos:
			self.success[3] = 1

		#print self.wrist1_final

		self.wrist1_error_prev = error

	def move_wrist2(self, cur_pos, req_pos):

		if self.command_mode == 1:
			req_pos =  -1.57
		if self.command_mode == 2:
			req_pos = 0

		error = req_pos-cur_pos
		derivative = error-self.wrist2_error_prev
		self.wrist2_final = error+derivative

		if self.wrist2_final > 1:
			self.wrist2_final = 1
		elif self.wrist2_final < -1:
			self.wrist2_final = -1
		elif self.wrist2_final > 0 and self.wrist2_final < 0.08:
			self.wrist2_final = 0.08
		elif self.wrist2_final < 0 and self.wrist2_final > -0.08:
			self.wrist2_final = -0.08

		if cur_pos == req_pos:
			self.success[4] = 1

		#print self.wrist2_final

		self.wrist2_error_prev = error

	def move_wrist3(self, cur_pos, req_pos):

		if self.command_mode == 1:
			req_pos = 0
		if self.command_mode == 2:
			req_pos = 0

		error = req_pos-cur_pos
		derivative = error-self.wrist3_error_prev
		self.wrist3_final = error+derivative

		if self.wrist3_final > 1:
			self.wrist3_final = 1
		elif self.wrist3_final < -1:
			self.wrist3_final = -1
		elif self.wrist3_final > 0 and self.wrist3_final < 0.08:
			self.wrist3_final = 0.08
		elif self.wrist3_final < 0 and self.wrist3_final > -0.08:
			self.wrist3_final = -0.08

		if cur_pos == req_pos:
			self.success[5] = 1

		#print cur_pos, req_pos

		self.wrist3_error_prev = error

	def cb_publish(self, event):
		if len(self.req) == 0:
			setpoint = [0.0]*6
			setp_msg = Setpoint()
			setp_msg.setpoint = setpoint
			setp_msg.type = Setpoint.TYPE_JOINT_VELOCITY
			self.pub_setpoint.publish(setp_msg)
		else:
			setpoint = [0.0]*6
			setpoint[0] = self.base_final*0.35
			setpoint[1] = self.shoulder_final*0.35
			setpoint[2] = self.elbow_final*0.35
			setpoint[3] = self.wrist1_final*0.35
			setpoint[4] = self.wrist2_final*0.35
			setpoint[5] = self.wrist3_final*0.35
			setp_msg = Setpoint()
			setp_msg.setpoint = setpoint
			setp_msg.type = Setpoint.TYPE_JOINT_VELOCITY
			self.pub_setpoint.publish(setp_msg)

if __name__ == '__main__':
	rospy.init_node('limb_manager')
	limb = Limb()

	try:              
		rospy.spin()

	except rospy.ROSInterruptException:
		pass