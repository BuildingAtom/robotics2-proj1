;; Auto-generated. Do not edit!


(when (boundp 'proj1robot_msgs::GPS)
  (if (not (find-package "PROJ1ROBOT_MSGS"))
    (make-package "PROJ1ROBOT_MSGS"))
  (shadow 'GPS (find-package "PROJ1ROBOT_MSGS")))
(unless (find-package "PROJ1ROBOT_MSGS::GPS")
  (make-package "PROJ1ROBOT_MSGS::GPS"))

(in-package "ROS")
;;//! \htmlinclude GPS.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass proj1robot_msgs::GPS
  :super ros::object
  :slots (_header _beacon00 _beacon01 _beacon10 _beacon11 ))

(defmethod proj1robot_msgs::GPS
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:beacon00 __beacon00) 0.0)
    ((:beacon01 __beacon01) 0.0)
    ((:beacon10 __beacon10) 0.0)
    ((:beacon11 __beacon11) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _beacon00 (float __beacon00))
   (setq _beacon01 (float __beacon01))
   (setq _beacon10 (float __beacon10))
   (setq _beacon11 (float __beacon11))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:beacon00
   (&optional __beacon00)
   (if __beacon00 (setq _beacon00 __beacon00)) _beacon00)
  (:beacon01
   (&optional __beacon01)
   (if __beacon01 (setq _beacon01 __beacon01)) _beacon01)
  (:beacon10
   (&optional __beacon10)
   (if __beacon10 (setq _beacon10 __beacon10)) _beacon10)
  (:beacon11
   (&optional __beacon11)
   (if __beacon11 (setq _beacon11 __beacon11)) _beacon11)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _beacon00
    8
    ;; float64 _beacon01
    8
    ;; float64 _beacon10
    8
    ;; float64 _beacon11
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _beacon00
       (sys::poke _beacon00 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _beacon01
       (sys::poke _beacon01 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _beacon10
       (sys::poke _beacon10 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _beacon11
       (sys::poke _beacon11 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _beacon00
     (setq _beacon00 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _beacon01
     (setq _beacon01 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _beacon10
     (setq _beacon10 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _beacon11
     (setq _beacon11 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get proj1robot_msgs::GPS :md5sum-) "6a88cfec89b2d3899365b3c2815c9280")
(setf (get proj1robot_msgs::GPS :datatype-) "proj1robot_msgs/GPS")
(setf (get proj1robot_msgs::GPS :definition-)
      "# A basic message for the local GPS beacons.

Header header       # Reference shenanigans

float64 beacon00    # Distance [m] from the beacon at [0,0,4]
float64 beacon01    # Distance [m] from the beacon at [0,10,4]
float64 beacon10    # Distance [m] from the beacon at [10,0,4]
float64 beacon11    # Distance [m] from the beacon at [10,10,4]
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



(provide :proj1robot_msgs/GPS "6a88cfec89b2d3899365b3c2815c9280")


