;; Auto-generated. Do not edit!


(when (boundp 'proj1robot_msgs::ControlInput)
  (if (not (find-package "PROJ1ROBOT_MSGS"))
    (make-package "PROJ1ROBOT_MSGS"))
  (shadow 'ControlInput (find-package "PROJ1ROBOT_MSGS")))
(unless (find-package "PROJ1ROBOT_MSGS::CONTROLINPUT")
  (make-package "PROJ1ROBOT_MSGS::CONTROLINPUT"))

(in-package "ROS")
;;//! \htmlinclude ControlInput.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass proj1robot_msgs::ControlInput
  :super ros::object
  :slots (_header _forward _turn ))

(defmethod proj1robot_msgs::ControlInput
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:forward __forward) 0.0)
    ((:turn __turn) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _forward (float __forward))
   (setq _turn (float __turn))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:forward
   (&optional __forward)
   (if __forward (setq _forward __forward)) _forward)
  (:turn
   (&optional __turn)
   (if __turn (setq _turn __turn)) _turn)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _forward
    8
    ;; float64 _turn
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _forward
       (sys::poke _forward (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _turn
       (sys::poke _turn (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _forward
     (setq _forward (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _turn
     (setq _turn (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get proj1robot_msgs::ControlInput :md5sum-) "78975323d29cfab13bfc997e1a08c9e1")
(setf (get proj1robot_msgs::ControlInput :datatype-) "proj1robot_msgs/ControlInput")
(setf (get proj1robot_msgs::ControlInput :definition-)
      "# Simple message to communicate intended control. Relayed by middleman.

Header header       # Reference shenanigans

float64 forward     # forward raw control input
float64 turn        # turn raw control input

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :proj1robot_msgs/ControlInput "78975323d29cfab13bfc997e1a08c9e1")


