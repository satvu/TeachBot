; Auto-generated. Do not edit!


(cl:in-package ur_dashboard_msgs-msg)


;//! \htmlinclude SetModeGoal.msg.html

(cl:defclass <SetModeGoal> (roslisp-msg-protocol:ros-message)
  ((target_robot_mode
    :reader target_robot_mode
    :initarg :target_robot_mode
    :type cl:fixnum
    :initform 0)
   (stop_program
    :reader stop_program
    :initarg :stop_program
    :type cl:boolean
    :initform cl:nil)
   (play_program
    :reader play_program
    :initarg :play_program
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetModeGoal (<SetModeGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModeGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModeGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_dashboard_msgs-msg:<SetModeGoal> is deprecated: use ur_dashboard_msgs-msg:SetModeGoal instead.")))

(cl:ensure-generic-function 'target_robot_mode-val :lambda-list '(m))
(cl:defmethod target_robot_mode-val ((m <SetModeGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-msg:target_robot_mode-val is deprecated.  Use ur_dashboard_msgs-msg:target_robot_mode instead.")
  (target_robot_mode m))

(cl:ensure-generic-function 'stop_program-val :lambda-list '(m))
(cl:defmethod stop_program-val ((m <SetModeGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-msg:stop_program-val is deprecated.  Use ur_dashboard_msgs-msg:stop_program instead.")
  (stop_program m))

(cl:ensure-generic-function 'play_program-val :lambda-list '(m))
(cl:defmethod play_program-val ((m <SetModeGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-msg:play_program-val is deprecated.  Use ur_dashboard_msgs-msg:play_program instead.")
  (play_program m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModeGoal>) ostream)
  "Serializes a message object of type '<SetModeGoal>"
  (cl:let* ((signed (cl:slot-value msg 'target_robot_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_program) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'play_program) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModeGoal>) istream)
  "Deserializes a message object of type '<SetModeGoal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_robot_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'stop_program) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'play_program) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModeGoal>)))
  "Returns string type for a message object of type '<SetModeGoal>"
  "ur_dashboard_msgs/SetModeGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModeGoal)))
  "Returns string type for a message object of type 'SetModeGoal"
  "ur_dashboard_msgs/SetModeGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModeGoal>)))
  "Returns md5sum for a message object of type '<SetModeGoal>"
  "6832df07338535cc06b3835f89ba9555")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModeGoal)))
  "Returns md5sum for a message object of type 'SetModeGoal"
  "6832df07338535cc06b3835f89ba9555")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModeGoal>)))
  "Returns full string definition for message of type '<SetModeGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# This action is for setting the robot into a desired mode (e.g. RUNNING) and safety mode into a~%# non-critical state (e.g. NORMAL or REDUCED), for example after a safety incident happened.~%~%# goal~%int8 target_robot_mode~%~%# Stop program execution before restoring the target mode. Can be used together with 'play_program'.~%bool stop_program~%~%# Play the currently loaded program after target mode is reached.#~%# NOTE: Requesting mode RUNNING in combination with this will make the robot continue the motion it~%# was doing before. This might probably lead into the same problem (protective stop, EM-Stop due to~%# faulty motion, etc.) If you want to be safe, set the 'stop_program' flag below and manually play~%# the program after robot state is returned to normal.~%# This flag will only be used when requesting mode RUNNING~%bool play_program~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModeGoal)))
  "Returns full string definition for message of type 'SetModeGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# This action is for setting the robot into a desired mode (e.g. RUNNING) and safety mode into a~%# non-critical state (e.g. NORMAL or REDUCED), for example after a safety incident happened.~%~%# goal~%int8 target_robot_mode~%~%# Stop program execution before restoring the target mode. Can be used together with 'play_program'.~%bool stop_program~%~%# Play the currently loaded program after target mode is reached.#~%# NOTE: Requesting mode RUNNING in combination with this will make the robot continue the motion it~%# was doing before. This might probably lead into the same problem (protective stop, EM-Stop due to~%# faulty motion, etc.) If you want to be safe, set the 'stop_program' flag below and manually play~%# the program after robot state is returned to normal.~%# This flag will only be used when requesting mode RUNNING~%bool play_program~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModeGoal>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModeGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModeGoal
    (cl:cons ':target_robot_mode (target_robot_mode msg))
    (cl:cons ':stop_program (stop_program msg))
    (cl:cons ':play_program (play_program msg))
))
