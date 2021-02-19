; Auto-generated. Do not edit!


(cl:in-package proj1robot_msgs-msg)


;//! \htmlinclude ControlInput.msg.html

(cl:defclass <ControlInput> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (forward
    :reader forward
    :initarg :forward
    :type cl:float
    :initform 0.0)
   (turn
    :reader turn
    :initarg :turn
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlInput (<ControlInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proj1robot_msgs-msg:<ControlInput> is deprecated: use proj1robot_msgs-msg:ControlInput instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proj1robot_msgs-msg:header-val is deprecated.  Use proj1robot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'forward-val :lambda-list '(m))
(cl:defmethod forward-val ((m <ControlInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proj1robot_msgs-msg:forward-val is deprecated.  Use proj1robot_msgs-msg:forward instead.")
  (forward m))

(cl:ensure-generic-function 'turn-val :lambda-list '(m))
(cl:defmethod turn-val ((m <ControlInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proj1robot_msgs-msg:turn-val is deprecated.  Use proj1robot_msgs-msg:turn instead.")
  (turn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlInput>) ostream)
  "Serializes a message object of type '<ControlInput>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'forward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'turn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlInput>) istream)
  "Deserializes a message object of type '<ControlInput>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forward) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'turn) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlInput>)))
  "Returns string type for a message object of type '<ControlInput>"
  "proj1robot_msgs/ControlInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlInput)))
  "Returns string type for a message object of type 'ControlInput"
  "proj1robot_msgs/ControlInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlInput>)))
  "Returns md5sum for a message object of type '<ControlInput>"
  "78975323d29cfab13bfc997e1a08c9e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlInput)))
  "Returns md5sum for a message object of type 'ControlInput"
  "78975323d29cfab13bfc997e1a08c9e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlInput>)))
  "Returns full string definition for message of type '<ControlInput>"
  (cl:format cl:nil "# Simple message to communicate intended control. Relayed by middleman.~%~%Header header       # Reference shenanigans~%~%float64 forward     # forward raw control input~%float64 turn        # turn raw control input~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlInput)))
  "Returns full string definition for message of type 'ControlInput"
  (cl:format cl:nil "# Simple message to communicate intended control. Relayed by middleman.~%~%Header header       # Reference shenanigans~%~%float64 forward     # forward raw control input~%float64 turn        # turn raw control input~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlInput>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlInput>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlInput
    (cl:cons ':header (header msg))
    (cl:cons ':forward (forward msg))
    (cl:cons ':turn (turn msg))
))
