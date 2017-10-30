; Auto-generated. Do not edit!


(cl:in-package ucl_drone-msg)


;//! \htmlinclude TargetDetected.msg.html

(cl:defclass <TargetDetected> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type ucl_drone-msg:Pose3D
    :initform (cl:make-instance 'ucl_drone-msg:Pose3D))
   (navdata
    :reader navdata
    :initarg :navdata
    :type ardrone_autonomy-msg:Navdata
    :initform (cl:make-instance 'ardrone_autonomy-msg:Navdata))
   (img_point
    :reader img_point
    :initarg :img_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (world_point
    :reader world_point
    :initarg :world_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TargetDetected (<TargetDetected>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetDetected>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetDetected)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-msg:<TargetDetected> is deprecated: use ucl_drone-msg:TargetDetected instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TargetDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:header-val is deprecated.  Use ucl_drone-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TargetDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:pose-val is deprecated.  Use ucl_drone-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'navdata-val :lambda-list '(m))
(cl:defmethod navdata-val ((m <TargetDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:navdata-val is deprecated.  Use ucl_drone-msg:navdata instead.")
  (navdata m))

(cl:ensure-generic-function 'img_point-val :lambda-list '(m))
(cl:defmethod img_point-val ((m <TargetDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:img_point-val is deprecated.  Use ucl_drone-msg:img_point instead.")
  (img_point m))

(cl:ensure-generic-function 'world_point-val :lambda-list '(m))
(cl:defmethod world_point-val ((m <TargetDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:world_point-val is deprecated.  Use ucl_drone-msg:world_point instead.")
  (world_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetDetected>) ostream)
  "Serializes a message object of type '<TargetDetected>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'navdata) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'world_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetDetected>) istream)
  "Deserializes a message object of type '<TargetDetected>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'navdata) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'world_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetDetected>)))
  "Returns string type for a message object of type '<TargetDetected>"
  "ucl_drone/TargetDetected")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetDetected)))
  "Returns string type for a message object of type 'TargetDetected"
  "ucl_drone/TargetDetected")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetDetected>)))
  "Returns md5sum for a message object of type '<TargetDetected>"
  "615a44da705823f9b18728cd0ad9c1aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetDetected)))
  "Returns md5sum for a message object of type 'TargetDetected"
  "615a44da705823f9b18728cd0ad9c1aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetDetected>)))
  "Returns full string definition for message of type '<TargetDetected>"
  (cl:format cl:nil "~%Header header~%Pose3D pose~%ardrone_autonomy/Navdata navdata # to be removed later~%geometry_msgs/Point img_point~%geometry_msgs/Point world_point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ucl_drone/Pose3D~%~%# This represents an estimate of a position and velocity in 3D space.~%# The pose in this message should be specified in an absolute coordinate frame.~%~%# ucl definition of a pose message.~%# TODO: unites?~%# Also add velocities for the controller purpose.~%# TODO: why exclude acceleration?~%~%Header header~%~%float64 x~%float64 y~%float64 z~%~%#float64 quatX~%#float64 quatY~%#float64 quatZ~%#float64 quatW~%~%float64 rotX~%float64 rotY~%float64 rotZ~%~%float64 xvel~%float64 yvel~%float64 zvel~%~%float64 rotXvel~%float64 rotYvel~%float64 rotZvel~%~%================================================================================~%MSG: ardrone_autonomy/Navdata~%Header header~%~%# Navdata including the ARDrone 2 specifica sensors~%# (magnetometer, barometer)~%~%# 0 means no battery, 100 means full battery~%float32 batteryPercent~%~%# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test~%# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping~%# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)~%uint32 state~%~%int32 magX~%int32 magY~%int32 magZ~%~%# pressure sensor~%int32 pressure~%~%# apparently, there was a temperature sensor added as well.~%int32 temp~%~%# wind sensing...~%float32 wind_speed~%float32 wind_angle~%float32 wind_comp_angle~%~%# left/right tilt in degrees (rotation about the X axis)~%float32 rotX~%~%# forward/backward tilt in degrees (rotation about the Y axis)~%float32 rotY~%~%# orientation in degrees (rotation about the Z axis)~%float32 rotZ~%~%# estimated altitude (cm)~%int32 altd~%~%# linear velocity (mm/sec)~%float32 vx~%~%# linear velocity (mm/sec)~%float32 vy~%~%# linear velocity (mm/sec)~%float32 vz~%~%#linear accelerations (unit: g)~%float32 ax~%float32 ay~%float32 az~%~%#motor commands (unit 0 to 255)~%uint8 motor1~%uint8 motor2~%uint8 motor3~%uint8 motor4~%~%#Tags in Vision Detectoion~%uint32 tags_count~%uint32[] tags_type~%uint32[] tags_xc~%uint32[] tags_yc~%uint32[] tags_width~%uint32[] tags_height~%float32[] tags_orientation~%float32[] tags_distance~%~%#time stamp~%float32 tm~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetDetected)))
  "Returns full string definition for message of type 'TargetDetected"
  (cl:format cl:nil "~%Header header~%Pose3D pose~%ardrone_autonomy/Navdata navdata # to be removed later~%geometry_msgs/Point img_point~%geometry_msgs/Point world_point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ucl_drone/Pose3D~%~%# This represents an estimate of a position and velocity in 3D space.~%# The pose in this message should be specified in an absolute coordinate frame.~%~%# ucl definition of a pose message.~%# TODO: unites?~%# Also add velocities for the controller purpose.~%# TODO: why exclude acceleration?~%~%Header header~%~%float64 x~%float64 y~%float64 z~%~%#float64 quatX~%#float64 quatY~%#float64 quatZ~%#float64 quatW~%~%float64 rotX~%float64 rotY~%float64 rotZ~%~%float64 xvel~%float64 yvel~%float64 zvel~%~%float64 rotXvel~%float64 rotYvel~%float64 rotZvel~%~%================================================================================~%MSG: ardrone_autonomy/Navdata~%Header header~%~%# Navdata including the ARDrone 2 specifica sensors~%# (magnetometer, barometer)~%~%# 0 means no battery, 100 means full battery~%float32 batteryPercent~%~%# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test~%# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping~%# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)~%uint32 state~%~%int32 magX~%int32 magY~%int32 magZ~%~%# pressure sensor~%int32 pressure~%~%# apparently, there was a temperature sensor added as well.~%int32 temp~%~%# wind sensing...~%float32 wind_speed~%float32 wind_angle~%float32 wind_comp_angle~%~%# left/right tilt in degrees (rotation about the X axis)~%float32 rotX~%~%# forward/backward tilt in degrees (rotation about the Y axis)~%float32 rotY~%~%# orientation in degrees (rotation about the Z axis)~%float32 rotZ~%~%# estimated altitude (cm)~%int32 altd~%~%# linear velocity (mm/sec)~%float32 vx~%~%# linear velocity (mm/sec)~%float32 vy~%~%# linear velocity (mm/sec)~%float32 vz~%~%#linear accelerations (unit: g)~%float32 ax~%float32 ay~%float32 az~%~%#motor commands (unit 0 to 255)~%uint8 motor1~%uint8 motor2~%uint8 motor3~%uint8 motor4~%~%#Tags in Vision Detectoion~%uint32 tags_count~%uint32[] tags_type~%uint32[] tags_xc~%uint32[] tags_yc~%uint32[] tags_width~%uint32[] tags_height~%float32[] tags_orientation~%float32[] tags_distance~%~%#time stamp~%float32 tm~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetDetected>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'navdata))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'world_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetDetected>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetDetected
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':navdata (navdata msg))
    (cl:cons ':img_point (img_point msg))
    (cl:cons ':world_point (world_point msg))
))
