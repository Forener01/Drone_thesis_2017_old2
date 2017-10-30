; Auto-generated. Do not edit!


(cl:in-package ucl_drone-msg)


;//! \htmlinclude KeyPoint.msg.html

(cl:defclass <KeyPoint> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (descriptor
    :reader descriptor
    :initarg :descriptor
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass KeyPoint (<KeyPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-msg:<KeyPoint> is deprecated: use ucl_drone-msg:KeyPoint instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <KeyPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:point-val is deprecated.  Use ucl_drone-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'descriptor-val :lambda-list '(m))
(cl:defmethod descriptor-val ((m <KeyPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:descriptor-val is deprecated.  Use ucl_drone-msg:descriptor instead.")
  (descriptor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyPoint>) ostream)
  "Serializes a message object of type '<KeyPoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'descriptor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'descriptor))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyPoint>) istream)
  "Deserializes a message object of type '<KeyPoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'descriptor) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'descriptor)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyPoint>)))
  "Returns string type for a message object of type '<KeyPoint>"
  "ucl_drone/KeyPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyPoint)))
  "Returns string type for a message object of type 'KeyPoint"
  "ucl_drone/KeyPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyPoint>)))
  "Returns md5sum for a message object of type '<KeyPoint>"
  "7471b927bc69af15f0d7edb3ca7158f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyPoint)))
  "Returns md5sum for a message object of type 'KeyPoint"
  "7471b927bc69af15f0d7edb3ca7158f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyPoint>)))
  "Returns full string definition for message of type '<KeyPoint>"
  (cl:format cl:nil "~%geometry_msgs/Point point~%float32[] descriptor # float 32 ?~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyPoint)))
  "Returns full string definition for message of type 'KeyPoint"
  (cl:format cl:nil "~%geometry_msgs/Point point~%float32[] descriptor # float 32 ?~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyPoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'descriptor) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyPoint
    (cl:cons ':point (point msg))
    (cl:cons ':descriptor (descriptor msg))
))
