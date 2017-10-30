; Auto-generated. Do not edit!


(cl:in-package ucl_drone-msg)


;//! \htmlinclude DroneRoles.msg.html

(cl:defclass <DroneRoles> (roslisp-msg-protocol:ros-message)
  ((roles
    :reader roles
    :initarg :roles
    :type (cl:vector ucl_drone-msg:DroneRole)
   :initform (cl:make-array 0 :element-type 'ucl_drone-msg:DroneRole :initial-element (cl:make-instance 'ucl_drone-msg:DroneRole))))
)

(cl:defclass DroneRoles (<DroneRoles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneRoles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneRoles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-msg:<DroneRoles> is deprecated: use ucl_drone-msg:DroneRoles instead.")))

(cl:ensure-generic-function 'roles-val :lambda-list '(m))
(cl:defmethod roles-val ((m <DroneRoles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:roles-val is deprecated.  Use ucl_drone-msg:roles instead.")
  (roles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneRoles>) ostream)
  "Serializes a message object of type '<DroneRoles>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'roles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'roles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneRoles>) istream)
  "Deserializes a message object of type '<DroneRoles>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'roles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'roles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ucl_drone-msg:DroneRole))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneRoles>)))
  "Returns string type for a message object of type '<DroneRoles>"
  "ucl_drone/DroneRoles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneRoles)))
  "Returns string type for a message object of type 'DroneRoles"
  "ucl_drone/DroneRoles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneRoles>)))
  "Returns md5sum for a message object of type '<DroneRoles>"
  "28014a7bc2067361e079b10f5ccfb8ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneRoles)))
  "Returns md5sum for a message object of type 'DroneRoles"
  "28014a7bc2067361e079b10f5ccfb8ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneRoles>)))
  "Returns full string definition for message of type '<DroneRoles>"
  (cl:format cl:nil "~%# This message contains a list of all the current drone roles, formulated by the~%# multi-agent strategy and read by each drone's IA.~%~%# Role list~%ucl_drone/DroneRole[] roles~%~%================================================================================~%MSG: ucl_drone/DroneRole~%~%# This message contains the object drone role, formulated by the multi-agent~%# strategy and read by each drone's IA.~%~%# Drone name~%string name~%~%# Role code~%float64 role~%~%# List of input-output topic names. Refer to the role code for convention about~%# the specific use of each field~%string[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneRoles)))
  "Returns full string definition for message of type 'DroneRoles"
  (cl:format cl:nil "~%# This message contains a list of all the current drone roles, formulated by the~%# multi-agent strategy and read by each drone's IA.~%~%# Role list~%ucl_drone/DroneRole[] roles~%~%================================================================================~%MSG: ucl_drone/DroneRole~%~%# This message contains the object drone role, formulated by the multi-agent~%# strategy and read by each drone's IA.~%~%# Drone name~%string name~%~%# Role code~%float64 role~%~%# List of input-output topic names. Refer to the role code for convention about~%# the specific use of each field~%string[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneRoles>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'roles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneRoles>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneRoles
    (cl:cons ':roles (roles msg))
))
