# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ucl_drone/ProcessedImageMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import sensor_msgs.msg
import geometry_msgs.msg
import ucl_drone.msg
import std_msgs.msg

class ProcessedImageMsg(genpy.Message):
  _md5sum = "9c449724f5d57d4b68e130c4e2535fe9"
  _type = "ucl_drone/ProcessedImageMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
KeyPoint[] keypoints
Pose3D pose
sensor_msgs/Image image
bool target_detected
geometry_msgs/Point[] target_points # corners and center

================================================================================
MSG: ucl_drone/KeyPoint

geometry_msgs/Point point
float32[] descriptor # float 32 ?

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: ucl_drone/Pose3D

# This represents an estimate of a position and velocity in 3D space.
# The pose in this message should be specified in an absolute coordinate frame.

# ucl definition of a pose message.
# TODO: unites?
# Also add velocities for the controller purpose.
# TODO: why exclude acceleration?

Header header

float64 x
float64 y
float64 z

#float64 quatX
#float64 quatY
#float64 quatZ
#float64 quatW

float64 rotX
float64 rotY
float64 rotZ

float64 xvel
float64 yvel
float64 zvel

float64 rotXvel
float64 rotYvel
float64 rotZvel

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
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
"""
  __slots__ = ['keypoints','pose','image','target_detected','target_points']
  _slot_types = ['ucl_drone/KeyPoint[]','ucl_drone/Pose3D','sensor_msgs/Image','bool','geometry_msgs/Point[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       keypoints,pose,image,target_detected,target_points

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ProcessedImageMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.keypoints is None:
        self.keypoints = []
      if self.pose is None:
        self.pose = ucl_drone.msg.Pose3D()
      if self.image is None:
        self.image = sensor_msgs.msg.Image()
      if self.target_detected is None:
        self.target_detected = False
      if self.target_points is None:
        self.target_points = []
    else:
      self.keypoints = []
      self.pose = ucl_drone.msg.Pose3D()
      self.image = sensor_msgs.msg.Image()
      self.target_detected = False
      self.target_points = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.keypoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.keypoints:
        _v1 = val1.point
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.descriptor)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(struct.pack(pattern, *val1.descriptor))
      _x = self
      buff.write(_struct_3I.pack(_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs))
      _x = self.pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_12d3I.pack(_x.pose.x, _x.pose.y, _x.pose.z, _x.pose.rotX, _x.pose.rotY, _x.pose.rotZ, _x.pose.xvel, _x.pose.yvel, _x.pose.zvel, _x.pose.rotXvel, _x.pose.rotYvel, _x.pose.rotZvel, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs))
      _x = self.image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.image.height, _x.image.width))
      _x = self.image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.image.is_bigendian, _x.image.step))
      _x = self.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.target_detected))
      length = len(self.target_points)
      buff.write(_struct_I.pack(length))
      for val1 in self.target_points:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.keypoints is None:
        self.keypoints = None
      if self.pose is None:
        self.pose = ucl_drone.msg.Pose3D()
      if self.image is None:
        self.image = sensor_msgs.msg.Image()
      if self.target_points is None:
        self.target_points = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.keypoints = []
      for i in range(0, length):
        val1 = ucl_drone.msg.KeyPoint()
        _v2 = val1.point
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.descriptor = struct.unpack(pattern, str[start:end])
        self.keypoints.append(val1)
      _x = self
      start = end
      end += 12
      (_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 108
      (_x.pose.x, _x.pose.y, _x.pose.z, _x.pose.rotX, _x.pose.rotY, _x.pose.rotZ, _x.pose.xvel, _x.pose.yvel, _x.pose.zvel, _x.pose.rotXvel, _x.pose.rotYvel, _x.pose.rotZvel, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs,) = _struct_12d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.image.height, _x.image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.image.encoding = str[start:end].decode('utf-8')
      else:
        self.image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.image.is_bigendian, _x.image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.data = str[start:end]
      start = end
      end += 1
      (self.target_detected,) = _struct_B.unpack(str[start:end])
      self.target_detected = bool(self.target_detected)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.target_points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.target_points.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.keypoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.keypoints:
        _v3 = val1.point
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.descriptor)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(val1.descriptor.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs))
      _x = self.pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_12d3I.pack(_x.pose.x, _x.pose.y, _x.pose.z, _x.pose.rotX, _x.pose.rotY, _x.pose.rotZ, _x.pose.xvel, _x.pose.yvel, _x.pose.zvel, _x.pose.rotXvel, _x.pose.rotYvel, _x.pose.rotZvel, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs))
      _x = self.image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.image.height, _x.image.width))
      _x = self.image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.image.is_bigendian, _x.image.step))
      _x = self.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.target_detected))
      length = len(self.target_points)
      buff.write(_struct_I.pack(length))
      for val1 in self.target_points:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.keypoints is None:
        self.keypoints = None
      if self.pose is None:
        self.pose = ucl_drone.msg.Pose3D()
      if self.image is None:
        self.image = sensor_msgs.msg.Image()
      if self.target_points is None:
        self.target_points = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.keypoints = []
      for i in range(0, length):
        val1 = ucl_drone.msg.KeyPoint()
        _v4 = val1.point
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.descriptor = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
        self.keypoints.append(val1)
      _x = self
      start = end
      end += 12
      (_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 108
      (_x.pose.x, _x.pose.y, _x.pose.z, _x.pose.rotX, _x.pose.rotY, _x.pose.rotZ, _x.pose.xvel, _x.pose.yvel, _x.pose.zvel, _x.pose.rotXvel, _x.pose.rotYvel, _x.pose.rotZvel, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs,) = _struct_12d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.image.height, _x.image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.image.encoding = str[start:end].decode('utf-8')
      else:
        self.image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.image.is_bigendian, _x.image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.data = str[start:end]
      start = end
      end += 1
      (self.target_detected,) = _struct_B.unpack(str[start:end])
      self.target_detected = bool(self.target_detected)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.target_points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.target_points.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
_struct_BI = struct.Struct("<BI")
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_12d3I = struct.Struct("<12d3I")
_struct_3d = struct.Struct("<3d")
