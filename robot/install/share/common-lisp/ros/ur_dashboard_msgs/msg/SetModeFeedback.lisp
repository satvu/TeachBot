; Auto-generated. Do not edit!


(cl:in-package ur_dashboard_msgs-msg)


;//! \htmlinclude SetModeFeedback.msg.html

(cl:defclass <SetModeFeedback> (roslisp-msg-protocol:ros-message)
  ((current_robot_mode
    :reader current_robot_mode
    :initarg :current_robot_mode
    :type cl:fixnum
    :initform 0)
   (current_safety_mode
    :reader current_safety_mode
    :initarg :current_safety_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetModeFeedback (<SetModeFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModeFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModeFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_dashboard_msgs-msg:<SetModeFeedback> is deprecated: use ur_dashboard_msgs-msg:SetModeFeedback instead.")))

(cl:ensure-generic-function 'current_robot_mode-val :lambda-list '(m))
(cl:defmethod current_robot_mode-val ((m <SetModeFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-msg:current_robot_mode-val is deprecated.  Use ur_dashboard_msgs-msg:current_robot_mode instead.")
  (current_robot_mode m))

(cl:ensure-generic-function 'current_safety_mode-val :lambda-list '(m))
(cl:defmethod current_safety_mode-val ((m <SetModeFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-msg:current_safety_mode-val is deprecated.  Use ur_dashboard_msgs-msg:current_safety_mode instead.")
  (current_safety_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModeFeedback>) ostream)
  "Serializes a message object of type '<SetModeFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'current_robot_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_safety_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModeFeedback>) istream)
  "Deserializes a message object of type '<SetModeFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_robot_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_safety_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModeFeedback>)))
  "Returns string type for a message object of type '<SetModeFeedback>"
  "ur_dashboard_msgs/SetModeFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModeFeedback)))
  "Returns string type for a message object of type 'SetModeFeedback"
  "ur_dashboard_msgs/SetModeFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModeFeedback>)))
  "Returns md5sum for a message object of type '<SetModeFeedback>"
  "d48caaba06f88e6be0ba90bf29940534")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModeFeedback)))
  "Returns md5sum for a message object of type 'SetModeFeedback"
  "d48caaba06f88e6be0ba90bf29940534")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModeFeedback>)))
  "Returns full string definition for message of type '<SetModeFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback~%int8 current_robot_mode~%int8 current_safety_mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModeFeedback)))
  "Returns full string definition for message of type 'SetModeFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback~%int8 current_robot_mode~%int8 current_safety_mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModeFeedback>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModeFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModeFeedback
    (cl:cons ':current_robot_mode (current_robot_mode msg))
    (cl:cons ':current_safety_mode (current_safety_mode msg))
))
