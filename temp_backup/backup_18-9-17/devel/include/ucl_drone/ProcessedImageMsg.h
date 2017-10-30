// Generated by gencpp from file ucl_drone/ProcessedImageMsg.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_PROCESSEDIMAGEMSG_H
#define UCL_DRONE_MESSAGE_PROCESSEDIMAGEMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ucl_drone/KeyPoint.h>
#include <ucl_drone/Pose3D.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

namespace ucl_drone
{
template <class ContainerAllocator>
struct ProcessedImageMsg_
{
  typedef ProcessedImageMsg_<ContainerAllocator> Type;

  ProcessedImageMsg_()
    : keypoints()
    , pose()
    , image()
    , target_detected(false)
    , target_points()  {
    }
  ProcessedImageMsg_(const ContainerAllocator& _alloc)
    : keypoints(_alloc)
    , pose(_alloc)
    , image(_alloc)
    , target_detected(false)
    , target_points(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::ucl_drone::KeyPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ucl_drone::KeyPoint_<ContainerAllocator> >::other >  _keypoints_type;
  _keypoints_type keypoints;

   typedef  ::ucl_drone::Pose3D_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _image_type;
  _image_type image;

   typedef uint8_t _target_detected_type;
  _target_detected_type target_detected;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _target_points_type;
  _target_points_type target_points;




  typedef boost::shared_ptr< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> const> ConstPtr;

}; // struct ProcessedImageMsg_

typedef ::ucl_drone::ProcessedImageMsg_<std::allocator<void> > ProcessedImageMsg;

typedef boost::shared_ptr< ::ucl_drone::ProcessedImageMsg > ProcessedImageMsgPtr;
typedef boost::shared_ptr< ::ucl_drone::ProcessedImageMsg const> ProcessedImageMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ucl_drone

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'ucl_drone': ['/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/laboinmastudent/Bureau/Drone_thesis_2017/Additional_packages/src/ardrone_autonomy/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c449724f5d57d4b68e130c4e2535fe9";
  }

  static const char* value(const ::ucl_drone::ProcessedImageMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c449724f5d57d4bULL;
  static const uint64_t static_value2 = 0x68e130c4e2535fe9ULL;
};

template<class ContainerAllocator>
struct DataType< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucl_drone/ProcessedImageMsg";
  }

  static const char* value(const ::ucl_drone::ProcessedImageMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
KeyPoint[] keypoints\n\
Pose3D pose\n\
sensor_msgs/Image image\n\
bool target_detected\n\
geometry_msgs/Point[] target_points # corners and center\n\
\n\
================================================================================\n\
MSG: ucl_drone/KeyPoint\n\
\n\
geometry_msgs/Point point\n\
float32[] descriptor # float 32 ?\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: ucl_drone/Pose3D\n\
\n\
# This represents an estimate of a position and velocity in 3D space.\n\
# The pose in this message should be specified in an absolute coordinate frame.\n\
\n\
# ucl definition of a pose message.\n\
# TODO: unites?\n\
# Also add velocities for the controller purpose.\n\
# TODO: why exclude acceleration?\n\
\n\
Header header\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
#float64 quatX\n\
#float64 quatY\n\
#float64 quatZ\n\
#float64 quatW\n\
\n\
float64 rotX\n\
float64 rotY\n\
float64 rotZ\n\
\n\
float64 xvel\n\
float64 yvel\n\
float64 zvel\n\
\n\
float64 rotXvel\n\
float64 rotYvel\n\
float64 rotZvel\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
";
  }

  static const char* value(const ::ucl_drone::ProcessedImageMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.keypoints);
      stream.next(m.pose);
      stream.next(m.image);
      stream.next(m.target_detected);
      stream.next(m.target_points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ProcessedImageMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucl_drone::ProcessedImageMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucl_drone::ProcessedImageMsg_<ContainerAllocator>& v)
  {
    s << indent << "keypoints[]" << std::endl;
    for (size_t i = 0; i < v.keypoints.size(); ++i)
    {
      s << indent << "  keypoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ucl_drone::KeyPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.keypoints[i]);
    }
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::ucl_drone::Pose3D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.image);
    s << indent << "target_detected: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.target_detected);
    s << indent << "target_points[]" << std::endl;
    for (size_t i = 0; i < v.target_points.size(); ++i)
    {
      s << indent << "  target_points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.target_points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCL_DRONE_MESSAGE_PROCESSEDIMAGEMSG_H
