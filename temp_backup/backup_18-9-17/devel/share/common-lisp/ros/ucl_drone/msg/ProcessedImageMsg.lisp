; Auto-generated. Do not edit!


(cl:in-package ucl_drone-msg)


;//! \htmlinclude ProcessedImageMsg.msg.html

(cl:defclass <ProcessedImageMsg> (roslisp-msg-protocol:ros-message)
  ((keypoints
    :reader keypoints
    :initarg :keypoints
    :type (cl:vector ucl_drone-msg:KeyPoint)
   :initform (cl:make-array 0 :element-type 'ucl_drone-msg:KeyPoint :initial-element (cl:make-instance 'ucl_drone-msg:KeyPoint)))
   (pose
    :reader pose
    :initarg :pose
    :type ucl_drone-msg:Pose3D
    :initform (cl:make-instance 'ucl_drone-msg:Pose3D))
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (target_detected
    :reader target_detected
    :initarg :target_detected
    :type cl:boolean
    :initform cl:nil)
   (target_points
    :reader target_points
    :initarg :target_points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ProcessedImageMsg (<ProcessedImageMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessedImageMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessedImageMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-msg:<ProcessedImageMsg> is deprecated: use ucl_drone-msg:ProcessedImageMsg instead.")))

(cl:ensure-generic-function 'keypoints-val :lambda-list '(m))
(cl:defmethod keypoints-val ((m <ProcessedImageMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:keypoints-val is deprecated.  Use ucl_drone-msg:keypoints instead.")
  (keypoints m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <ProcessedImageMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:pose-val is deprecated.  Use ucl_drone-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <ProcessedImageMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:image-val is deprecated.  Use ucl_drone-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'target_detected-val :lambda-list '(m))
(cl:defmethod target_detected-val ((m <ProcessedImageMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:target_detected-val is deprecated.  Use ucl_drone-msg:target_detected instead.")
  (target_detected m))

(cl:ensure-generic-function 'target_points-val :lambda-list '(m))
(cl:defmethod target_points-val ((m <ProcessedImageMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-msg:target_points-val is deprecated.  Use ucl_drone-msg:target_points instead.")
  (target_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessedImageMsg>) ostream)
  "Serializes a message object of type '<ProcessedImageMsg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'keypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'keypoints))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'target_detected) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'target_points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessedImageMsg>) istream)
  "Deserializes a message object of type '<ProcessedImageMsg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'keypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'keypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ucl_drone-msg:KeyPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
    (cl:setf (cl:slot-value msg 'target_detected) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessedImageMsg>)))
  "Returns string type for a message object of type '<ProcessedImageMsg>"
  "ucl_drone/ProcessedImageMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessedImageMsg)))
  "Returns string type for a message object of type 'ProcessedImageMsg"
  "ucl_drone/ProcessedImageMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessedImageMsg>)))
  "Returns md5sum for a message object of type '<ProcessedImageMsg>"
  "9c449724f5d57d4b68e130c4e2535fe9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessedImageMsg)))
  "Returns md5sum for a message object of type 'ProcessedImageMsg"
  "9c449724f5d57d4b68e130c4e2535fe9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessedImageMsg>)))
  "Returns full string definition for message of type '<ProcessedImageMsg>"
  (cl:format cl:nil "~%KeyPoint[] keypoints~%Pose3D pose~%sensor_msgs/Image image~%bool target_detected~%geometry_msgs/Point[] target_points # corners and center~%~%================================================================================~%MSG: ucl_drone/KeyPoint~%~%geometry_msgs/Point point~%float32[] descriptor # float 32 ?~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: ucl_drone/Pose3D~%~%# This represents an estimate of a position and velocity in 3D space.~%# The pose in this message should be specified in an absolute coordinate frame.~%~%# ucl definition of a pose message.~%# TODO: unites?~%# Also add velocities for the controller purpose.~%# TODO: why exclude acceleration?~%~%Header header~%~%float64 x~%float64 y~%float64 z~%~%#float64 quatX~%#float64 quatY~%#float64 quatZ~%#float64 quatW~%~%float64 rotX~%float64 rotY~%float64 rotZ~%~%float64 xvel~%float64 yvel~%float64 zvel~%~%float64 rotXvel~%float64 rotYvel~%float64 rotZvel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessedImageMsg)))
  "Returns full string definition for message of type 'ProcessedImageMsg"
  (cl:format cl:nil "~%KeyPoint[] keypoints~%Pose3D pose~%sensor_msgs/Image image~%bool target_detected~%geometry_msgs/Point[] target_points # corners and center~%~%================================================================================~%MSG: ucl_drone/KeyPoint~%~%geometry_msgs/Point point~%float32[] descriptor # float 32 ?~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: ucl_drone/Pose3D~%~%# This represents an estimate of a position and velocity in 3D space.~%# The pose in this message should be specified in an absolute coordinate frame.~%~%# ucl definition of a pose message.~%# TODO: unites?~%# Also add velocities for the controller purpose.~%# TODO: why exclude acceleration?~%~%Header header~%~%float64 x~%float64 y~%float64 z~%~%#float64 quatX~%#float64 quatY~%#float64 quatZ~%#float64 quatW~%~%float64 rotX~%float64 rotY~%float64 rotZ~%~%float64 xvel~%float64 yvel~%float64 zvel~%~%float64 rotXvel~%float64 rotYvel~%float64 rotZvel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessedImageMsg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'keypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessedImageMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessedImageMsg
    (cl:cons ':keypoints (keypoints msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':image (image msg))
    (cl:cons ':target_detected (target_detected msg))
    (cl:cons ':target_points (target_points msg))
))
