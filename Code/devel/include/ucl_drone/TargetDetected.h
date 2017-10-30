// Generated by gencpp from file ucl_drone/TargetDetected.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_TARGETDETECTED_H
#define UCL_DRONE_MESSAGE_TARGETDETECTED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <ucl_drone/Pose3D.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace ucl_drone
{
template <class ContainerAllocator>
struct TargetDetected_
{
  typedef TargetDetected_<ContainerAllocator> Type;

  TargetDetected_()
    : header()
    , pose()
    , navdata()
    , img_point()
    , world_point()  {
    }
  TargetDetected_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)
    , navdata(_alloc)
    , img_point(_alloc)
    , world_point(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::ucl_drone::Pose3D_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::ardrone_autonomy::Navdata_<ContainerAllocator>  _navdata_type;
  _navdata_type navdata;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _img_point_type;
  _img_point_type img_point;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _world_point_type;
  _world_point_type world_point;




  typedef boost::shared_ptr< ::ucl_drone::TargetDetected_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucl_drone::TargetDetected_<ContainerAllocator> const> ConstPtr;

}; // struct TargetDetected_

typedef ::ucl_drone::TargetDetected_<std::allocator<void> > TargetDetected;

typedef boost::shared_ptr< ::ucl_drone::TargetDetected > TargetDetectedPtr;
typedef boost::shared_ptr< ::ucl_drone::TargetDetected const> TargetDetectedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucl_drone::TargetDetected_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucl_drone::TargetDetected_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ucl_drone

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'ucl_drone': ['/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/opt/ros/indigo/share/ardrone_autonomy/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::TargetDetected_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::TargetDetected_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::TargetDetected_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::TargetDetected_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::TargetDetected_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::TargetDetected_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucl_drone::TargetDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "615a44da705823f9b18728cd0ad9c1aa";
  }

  static const char* value(const ::ucl_drone::TargetDetected_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x615a44da705823f9ULL;
  static const uint64_t static_value2 = 0xb18728cd0ad9c1aaULL;
};

template<class ContainerAllocator>
struct DataType< ::ucl_drone::TargetDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucl_drone/TargetDetected";
  }

  static const char* value(const ::ucl_drone::TargetDetected_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucl_drone::TargetDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
Header header\n\
Pose3D pose\n\
ardrone_autonomy/Navdata navdata # to be removed later\n\
geometry_msgs/Point img_point\n\
geometry_msgs/Point world_point\n\
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
MSG: ardrone_autonomy/Navdata\n\
Header header\n\
\n\
# Navdata including the ARDrone 2 specifica sensors\n\
# (magnetometer, barometer)\n\
\n\
# 0 means no battery, 100 means full battery\n\
float32 batteryPercent\n\
\n\
# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test\n\
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping\n\
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)\n\
uint32 state\n\
\n\
int32 magX\n\
int32 magY\n\
int32 magZ\n\
\n\
# pressure sensor\n\
int32 pressure\n\
\n\
# apparently, there was a temperature sensor added as well.\n\
int32 temp\n\
\n\
# wind sensing...\n\
float32 wind_speed\n\
float32 wind_angle\n\
float32 wind_comp_angle\n\
\n\
# left/right tilt in degrees (rotation about the X axis)\n\
float32 rotX\n\
\n\
# forward/backward tilt in degrees (rotation about the Y axis)\n\
float32 rotY\n\
\n\
# orientation in degrees (rotation about the Z axis)\n\
float32 rotZ\n\
\n\
# estimated altitude (cm)\n\
int32 altd\n\
\n\
# linear velocity (mm/sec)\n\
float32 vx\n\
\n\
# linear velocity (mm/sec)\n\
float32 vy\n\
\n\
# linear velocity (mm/sec)\n\
float32 vz\n\
\n\
#linear accelerations (unit: g)\n\
float32 ax\n\
float32 ay\n\
float32 az\n\
\n\
#motor commands (unit 0 to 255)\n\
uint8 motor1\n\
uint8 motor2\n\
uint8 motor3\n\
uint8 motor4\n\
\n\
#Tags in Vision Detectoion\n\
uint32 tags_count\n\
uint32[] tags_type\n\
uint32[] tags_xc\n\
uint32[] tags_yc\n\
uint32[] tags_width\n\
uint32[] tags_height\n\
float32[] tags_orientation\n\
float32[] tags_distance\n\
\n\
#time stamp\n\
float32 tm\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::ucl_drone::TargetDetected_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucl_drone::TargetDetected_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
      stream.next(m.navdata);
      stream.next(m.img_point);
      stream.next(m.world_point);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TargetDetected_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucl_drone::TargetDetected_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucl_drone::TargetDetected_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::ucl_drone::Pose3D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "navdata: ";
    s << std::endl;
    Printer< ::ardrone_autonomy::Navdata_<ContainerAllocator> >::stream(s, indent + "  ", v.navdata);
    s << indent << "img_point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.img_point);
    s << indent << "world_point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.world_point);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCL_DRONE_MESSAGE_TARGETDETECTED_H