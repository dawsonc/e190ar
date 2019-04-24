; Auto-generated. Do not edit!


(cl:in-package e190_bot-srv)


;//! \htmlinclude path_Service-request.msg.html

(cl:defclass <path_Service-request> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass path_Service-request (<path_Service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <path_Service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'path_Service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name e190_bot-srv:<path_Service-request> is deprecated: use e190_bot-srv:path_Service-request instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <path_Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader e190_bot-srv:current-val is deprecated.  Use e190_bot-srv:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <path_Service-request>) ostream)
  "Serializes a message object of type '<path_Service-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <path_Service-request>) istream)
  "Deserializes a message object of type '<path_Service-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<path_Service-request>)))
  "Returns string type for a service object of type '<path_Service-request>"
  "e190_bot/path_ServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'path_Service-request)))
  "Returns string type for a service object of type 'path_Service-request"
  "e190_bot/path_ServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<path_Service-request>)))
  "Returns md5sum for a message object of type '<path_Service-request>"
  "9d2f833ec7c0e39cfbf37318393587cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'path_Service-request)))
  "Returns md5sum for a message object of type 'path_Service-request"
  "9d2f833ec7c0e39cfbf37318393587cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<path_Service-request>)))
  "Returns full string definition for message of type '<path_Service-request>"
  (cl:format cl:nil "geometry_msgs/PoseStamped current~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'path_Service-request)))
  "Returns full string definition for message of type 'path_Service-request"
  (cl:format cl:nil "geometry_msgs/PoseStamped current~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <path_Service-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <path_Service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'path_Service-request
    (cl:cons ':current (current msg))
))
;//! \htmlinclude path_Service-response.msg.html

(cl:defclass <path_Service-response> (roslisp-msg-protocol:ros-message)
  ((next
    :reader next
    :initarg :next
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass path_Service-response (<path_Service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <path_Service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'path_Service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name e190_bot-srv:<path_Service-response> is deprecated: use e190_bot-srv:path_Service-response instead.")))

(cl:ensure-generic-function 'next-val :lambda-list '(m))
(cl:defmethod next-val ((m <path_Service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader e190_bot-srv:next-val is deprecated.  Use e190_bot-srv:next instead.")
  (next m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <path_Service-response>) ostream)
  "Serializes a message object of type '<path_Service-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <path_Service-response>) istream)
  "Deserializes a message object of type '<path_Service-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<path_Service-response>)))
  "Returns string type for a service object of type '<path_Service-response>"
  "e190_bot/path_ServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'path_Service-response)))
  "Returns string type for a service object of type 'path_Service-response"
  "e190_bot/path_ServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<path_Service-response>)))
  "Returns md5sum for a message object of type '<path_Service-response>"
  "9d2f833ec7c0e39cfbf37318393587cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'path_Service-response)))
  "Returns md5sum for a message object of type 'path_Service-response"
  "9d2f833ec7c0e39cfbf37318393587cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<path_Service-response>)))
  "Returns full string definition for message of type '<path_Service-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped next~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'path_Service-response)))
  "Returns full string definition for message of type 'path_Service-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped next~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <path_Service-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <path_Service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'path_Service-response
    (cl:cons ':next (next msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'path_Service)))
  'path_Service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'path_Service)))
  'path_Service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'path_Service)))
  "Returns string type for a service object of type '<path_Service>"
  "e190_bot/path_Service")