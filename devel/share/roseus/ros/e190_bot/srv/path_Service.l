;; Auto-generated. Do not edit!


(when (boundp 'e190_bot::path_Service)
  (if (not (find-package "E190_BOT"))
    (make-package "E190_BOT"))
  (shadow 'path_Service (find-package "E190_BOT")))
(unless (find-package "E190_BOT::PATH_SERVICE")
  (make-package "E190_BOT::PATH_SERVICE"))
(unless (find-package "E190_BOT::PATH_SERVICEREQUEST")
  (make-package "E190_BOT::PATH_SERVICEREQUEST"))
(unless (find-package "E190_BOT::PATH_SERVICERESPONSE")
  (make-package "E190_BOT::PATH_SERVICERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass e190_bot::path_ServiceRequest
  :super ros::object
  :slots (_current ))

(defmethod e190_bot::path_ServiceRequest
  (:init
   (&key
    ((:current __current) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _current __current)
   self)
  (:current
   (&rest __current)
   (if (keywordp (car __current))
       (send* _current __current)
     (progn
       (if __current (setq _current (car __current)))
       _current)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _current
    (send _current :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _current
       (send _current :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _current
     (send _current :deserialize buf ptr-) (incf ptr- (send _current :serialization-length))
   ;;
   self)
  )

(defclass e190_bot::path_ServiceResponse
  :super ros::object
  :slots (_next ))

(defmethod e190_bot::path_ServiceResponse
  (:init
   (&key
    ((:next __next) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _next __next)
   self)
  (:next
   (&rest __next)
   (if (keywordp (car __next))
       (send* _next __next)
     (progn
       (if __next (setq _next (car __next)))
       _next)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _next
    (send _next :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _next
       (send _next :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _next
     (send _next :deserialize buf ptr-) (incf ptr- (send _next :serialization-length))
   ;;
   self)
  )

(defclass e190_bot::path_Service
  :super ros::object
  :slots ())

(setf (get e190_bot::path_Service :md5sum-) "9d2f833ec7c0e39cfbf37318393587cd")
(setf (get e190_bot::path_Service :datatype-) "e190_bot/path_Service")
(setf (get e190_bot::path_Service :request) e190_bot::path_ServiceRequest)
(setf (get e190_bot::path_Service :response) e190_bot::path_ServiceResponse)

(defmethod e190_bot::path_ServiceRequest
  (:response () (instance e190_bot::path_ServiceResponse :init)))

(setf (get e190_bot::path_ServiceRequest :md5sum-) "9d2f833ec7c0e39cfbf37318393587cd")
(setf (get e190_bot::path_ServiceRequest :datatype-) "e190_bot/path_ServiceRequest")
(setf (get e190_bot::path_ServiceRequest :definition-)
      "geometry_msgs/PoseStamped current

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
geometry_msgs/PoseStamped next


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get e190_bot::path_ServiceResponse :md5sum-) "9d2f833ec7c0e39cfbf37318393587cd")
(setf (get e190_bot::path_ServiceResponse :datatype-) "e190_bot/path_ServiceResponse")
(setf (get e190_bot::path_ServiceResponse :definition-)
      "geometry_msgs/PoseStamped current

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
geometry_msgs/PoseStamped next


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :e190_bot/path_Service "9d2f833ec7c0e39cfbf37318393587cd")


