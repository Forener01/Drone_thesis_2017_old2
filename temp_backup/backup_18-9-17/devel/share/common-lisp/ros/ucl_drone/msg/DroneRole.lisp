; Auto-generated. Do not edit!


(cl:in-package ucl_drone-msg)


;//! \htmlinclude DroneRole.msg.html

(cl:defclass <DroneRole> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (role
    :reader role
    :initarg :role
    :type cl:float
    :initform 0.0)
   (params
    :reader params
    :initarg :params
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass DroneRole (<DroneRole>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneRole>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneRole)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-msg:<DroneRole> is deprecated: use ucl_drone-msg:DroneRole instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <DroneRole>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:name-val is deprecated.  Use ucl_drone-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'role-val :lambda-list '(m))
(cl:defmethod role-val ((m <DroneRole>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:role-val is deprecated.  Use ucl_drone-msg:role instead.")
  (role m))

(cl:ensure-generic-function 'params-val :lambda-list '(m))
(cl:defmethod params-val ((m <DroneRole>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:params-val is deprecated.  Use ucl_drone-msg:params instead.")
  (params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneRole>) ostream)
  "Serializes a message object of type '<DroneRole>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'role))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'params))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'params))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneRole>) istream)
  "Deserializes a message object of type '<DroneRole>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'role) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'params) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'params)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneRole>)))
  "Returns string type for a message object of type '<DroneRole>"
  "ucl_drone/DroneRole")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneRole)))
  "Returns string type for a message object of type 'DroneRole"
  "ucl_drone/DroneRole")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneRole>)))
  "Returns md5sum for a message object of type '<DroneRole>"
  "aa46bcf5f74539f0bbc0ce8ce5b59675")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneRole)))
  "Returns md5sum for a message object of type 'DroneRole"
  "aa46bcf5f74539f0bbc0ce8ce5b59675")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneRole>)))
  "Returns full string definition for message of type '<DroneRole>"
  (cl:format cl:nil "~%# This message contains the object drone role, formulated by the multi-agent~%# strategy and read by each drone's IA.~%~%# Drone name~%string name~%~%# Role code~%float64 role~%~%# List of input-output topic names. Refer to the role code for convention about~%# the specific use of each field~%string[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneRole)))
  "Returns full string definition for message of type 'DroneRole"
  (cl:format cl:nil "~%# This message contains the object drone role, formulated by the multi-agent~%# strategy and read by each drone's IA.~%~%# Drone name~%string name~%~%# Role code~%float64 role~%~%# List of input-output topic names. Refer to the role code for convention about~%# the specific use of each field~%string[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneRole>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'params) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneRole>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneRole
    (cl:cons ':name (name msg))
    (cl:cons ':role (role msg))
    (cl:cons ':params (params msg))
))
