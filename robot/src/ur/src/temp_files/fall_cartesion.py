    def cb_GoToCartesianPose(self, req):
        '''
        Receives "GoToCartesianPose" message and based on which paramters are given, perform different kinematic
        operations.
        '''
        # Evaluate all of the parameters (defined in this file, passed through as string in the action)
        joint_angles = eval(req.joint_angles)
        relative_pose = eval(req.relative_pose)
        endpoint_pose = eval(req.endpoint_pose)
        position = eval(req.position)
        orientation = eval(req.orientation)

        # Other values defined in Sawyer (see limb_plus.py)
        # TODO: See if we actually need these values (then check if accurate) and how to use when we don't have velocity interface
        in_tip_frame = False
        tip_name = self.endlink
        linear_speed = 0.6
        linear_accel = 0.6
        rotational_speed = 1.57
        rotational_accel = 1.57 
        timeout = None

        # hold 6 values, all joint angles 
        converted_joint_angles = []

        # set up result 
        result_GoToCartesianPose = GoToCartesianPoseResult()
        result_GoToCartesianPose.is_done = False

        if joint_angles and len(joint_angles) != len(JOINT_NAMES):
            rospy.logerr('len(joint_angles) does not match len(JOINT_NAMES)!')
            return None

        if (position is None and orientation is None and relative_pose is None and endpoint_pose is None):
            if joint_angles:
                # Forward Kinematics <-- comment from the Sawyer package, this section should be fk, but seems this is just a goToJointAngles
                # in response to above comment, actually need to do kinematics, the below was used in Sawyer b/c 7th degree of freedom
                joint_input = GoToJointAngles()
                joint_input.name = req.joint_angles
                # TODO: do I have to do an action here or can I just pass in input as I am currently doing?
                self.cb_GoToJointAngles(joint_input)

                # successfully completed action, return to browser
                result_GoToCartesianPose.is_done = True
                self.GoToCartesianPoseAct.set_succeeded(result_GoToCartesianPose)

                # escape function
                return None 

            else:
                rospy.loginfo("No Cartesian pose or joint angles given.")
                return None
        else:
            # TODO: Figure out what this chunk is about. There is not a self.tip_state as far as I can see

			# endpoint_state = self.tip_state(tip_name)
			# if endpoint_state is None:
			# 	rospy.logerr('Endpoint state not found with tip name %s', tip_name)
			# 	return None
			# pose = endpoint_state.pose

            if relative_pose is not None:
                if len(relative_pose) != 6:
                    rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
                    return None
                # create kdl frame from relative pose
                rot = kdl.Rotation.RPY(relative_pose[3], relative_pose[4], relative_pose[5])
                trans = kdl.Vector(relative_pose[0], relative_pose[1], relative_pose[2])
                f2 = kdl.Frame(rot, trans)

                # TODO: base frame or end effector frame, figure out ik for this 
                # and convert the result back to a pose message
                if in_tip_frame:
                    # end effector frame - line below is from Sawyer limb_plus so not sure what this translates to
                    # pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
                    return
                else:
                    # base frame
                    q_out=JntArray(6)
                    # TODO: how to check the result and make sure it's fine, there was no checking in the limb_plus version
                    # http://docs.ros.org/jade/api/orocos_kdl/html/classKDL_1_1ChainIkSolverPos__NR.html 
                    # above is the C++
                    ik_result = self.ik.CartToJnt(self.curr_pos, f2, q_out)
                    converted_joint_angles = q_out
                    
            else:
                if endpoint_pose is None:
                    # create kdl frame from given
                    if position is not None and len(position) == 3:
                        trans = kdl.Vector(position[0], position[1], position[2])
                        
                    if orientation is not None and len(orientation) == 4:
                        rot = kdl.Rotation.RPY(orientation[0], orientation[1], orientation[2])
                        # TODO: Figure out where to put wrench (orientation[3]) from KDL docs 
                        # only operation I can find with wrench is that you multiply it with frame? it represents force/torque so maybe s
                        # should be used with a velocity interface instead of a position interface?

                    f2 = kdl.Frame(rot, trans)
                    q_out=JntArray(6)
                    # TODO: do I need to do any checking w/ value of ik_result? See comment at line 219 above
                    ik_result = self.ik.CartToJnt(self.curr_pos, f2, q_out)
                    converted_joint_angles = q_out
                    
                else:
                    trans = kdl.Vector(endpoint_pose['position'][0], endpoint_pose['position'][1], endpoint_pose['position'][2])
                    rot = kdl.Rotation.RPY(endpoint_pose['orientation'][0], endpoint_pose['orientation'][1], endpoint_pose['orientation'][2])
                    # TODO: Figure out where to put wrench (orientation[3])
                    q_out=JntArray(6)
                    # TODO: do I need to do any checking w/ value of ik_result?
                    ik_result = self.ik.CartToJnt(self.curr_pos, f2, q_out)
                    converted_joint_angles = q_out

            if not joint_angles:
                # Set the new joint angle to go to
                joint_input = GoToJointAngles()
                joint_input.j0pos = converted_joint_angles[0]
                joint_input.j1pos = converted_joint_angles[1]
                joint_input.j2pos = converted_joint_angles[2]
                joint_input.j3pos = converted_joint_angles[3]
                joint_input.j4pos = converted_joint_angles[4]
                joint_input.j5pos = converted_joint_angles[5]

                self.cb_GoToJointAngles(joint_input)
                
                # successfully completed action, return to browser
                result_GoToCartesianPose.is_done = True
                self.GoToCartesianPoseAct.set_succeeded(result_GoToCartesianPose)
                
            return None 