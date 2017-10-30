; Auto-generated. Do not edit!


(cl:in-package ucl_drone-msg)


;//! \htmlinclude PoseRef.msg.html

(cl:defclass <PoseRef> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (rotZ
    :reader rotZ
    :initarg :rotZ
    :type cl:float
    :initform 0.0)
   (landAndStop
    :reader landAndStop
    :initarg :landAndStop
    :type cl:boolean
    :initform cl:nil)
   (takeoffAndStart
    :reader takeoffAndStart
    :initarg :takeoffAndStart
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PoseRef (<PoseRef>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseRef>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseRef)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-msg:<PoseRef> is deprecated: use ucl_drone-msg:PoseRef instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:header-val is deprecated.  Use ucl_drone-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:x-val is deprecated.  Use ucl_drone-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:y-val is deprecated.  Use ucl_drone-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:z-val is deprecated.  Use ucl_drone-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'rotZ-val :lambda-list '(m))
(cl:defmethod rotZ-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:rotZ-val is deprecated.  Use ucl_drone-msg:rotZ instead.")
  (rotZ m))

(cl:ensure-generic-function 'landAndStop-val :lambda-list '(m))
(cl:defmethod landAndStop-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:landAndStop-val is deprecated.  Use ucl_drone-msg:landAndStop instead.")
  (landAndStop m))

(cl:ensure-generic-function 'takeoffAndStart-val :lambda-list '(m))
(cl:defmethod takeoffAndStart-val ((m <PoseRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:takeoffAndStart-val is deprecated.  Use ucl_drone-msg:takeoffAndStart instead.")
  (takeoffAndStart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseRef>) ostream)
  "Serializes a message object of type '<PoseRef>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'landAndStop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'takeoffAndStart) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseRef>) istream)
  "Deserializes a message object of type '<PoseRef>"
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
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotZ) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'landAndStop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'takeoffAndStart) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseRef>)))
  "Returns string type for a message object of type '<PoseRef>"
  "ucl_drone/PoseRef")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseRef)))
  "Returns string type for a message object of type 'PoseRef"
  "ucl_drone/PoseRef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseRef>)))
  "Returns md5sum for a message object of type '<PoseRef>"
  "d40f9037129845d22f4a15b255ed3109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseRef)))
  "Returns md5sum for a message object of type 'PoseRef"
  "d40f9037129845d22f4a15b255ed3109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseRef>)))
  "Returns full string definition for message of type '<PoseRef>"
  (cl:format cl:nil "~%# This represents an estimate of a position and velocity in 3D space.~%# The pose in this message should be specified in an absolute coordinate frame.~%~%# ucl definition of a desired pose message.~%~%Header header~%~%float64 x~%float64 y~%float64 z~%~%float64 rotZ~%~%bool landAndStop~%bool takeoffAndStart~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseRef)))
  "Returns full string definition for message of type 'PoseRef"
  (cl:format cl:nil "~%# This represents an estimate of a position and velocity in 3D space.~%# The pose in this message should be specified in an absolute coordinate frame.~%~%# ucl definition of a desired pose message.~%~%Header header~%~%float64 x~%float64 y~%float64 z~%~%float64 rotZ~%~%bool landAndStop~%bool takeoffAndStart~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseRef>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseRef>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseRef
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':rotZ (rotZ msg))
    (cl:cons ':landAndStop (landAndStop msg))
    (cl:cons ':takeoffAndStart (takeoffAndStart msg))
))
