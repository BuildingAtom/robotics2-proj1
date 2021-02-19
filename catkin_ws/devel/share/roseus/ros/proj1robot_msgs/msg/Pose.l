;; Auto-generated. Do not edit!


(when (boundp 'proj1robot_msgs::Pose)
  (if (not (find-package "PROJ1ROBOT_MSGS"))
    (make-package "PROJ1ROBOT_MSGS"))
  (shadow 'Pose (find-package "PROJ1ROBOT_MSGS")))
(unless (find-package "PROJ1ROBOT_MSGS::POSE")
  (make-package "PROJ1ROBOT_MSGS::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass proj1robot_msgs::Pose
  :super ros::object
  :slots (_header _x _y _theta ))

(defmethod proj1robot_msgs::Pose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get proj1robot_msgs::Pose :md5sum-) "013fad68aad141d90a055e21fb791e63")
(setf (get proj1robot_msgs::Pose :datatype-) "proj1robot_msgs/Pose")
(setf (get proj1robot_msgs::Pose :definition-)
      "# A basic message for the pose (simplified for 2D)
# Estimates the pose of the robot's origin

Header header       # Reference shenanigans (frame is always world)

float64 x           # x position in world frame in meters (inf if N/A)
float64 y           # y position in world frame in meters (inf if N/A)
float64 theta       # theta orientation in world frame in radians (inf if N/A)

# Row-major 3x3 covariance of the x, y, and theta.
#float64[9]　covariance

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



(provide :proj1robot_msgs/Pose "013fad68aad141d90a055e21fb791e63")


