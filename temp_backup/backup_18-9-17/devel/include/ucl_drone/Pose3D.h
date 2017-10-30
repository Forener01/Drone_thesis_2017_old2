// Generated by gencpp from file ucl_drone/Pose3D.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_POSE3D_H
#define UCL_DRONE_MESSAGE_POSE3D_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ucl_drone
{
template <class ContainerAllocator>
struct Pose3D_
{
  typedef Pose3D_<ContainerAllocator> Type;

  Pose3D_()
    : header()
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rotX(0.0)
    , rotY(0.0)
    , rotZ(0.0)
    , xvel(0.0)
    , yvel(0.0)
    , zvel(0.0)
    , rotXvel(0.0)
    , rotYvel(0.0)
    , rotZvel(0.0)  {
    }
  Pose3D_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rotX(0.0)
    , rotY(0.0)
    , rotZ(0.0)
    , xvel(0.0)
    , yvel(0.0)
    , zvel(0.0)
    , rotXvel(0.0)
    , rotYvel(0.0)
    , rotZvel(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _rotX_type;
  _rotX_type rotX;

   typedef double _rotY_type;
  _rotY_type rotY;

   typedef double _rotZ_type;
  _rotZ_type rotZ;

   typedef double _xvel_type;
  _xvel_type xvel;

   typedef double _yvel_type;
  _yvel_type yvel;

   typedef double _zvel_type;
  _zvel_type zvel;

   typedef double _rotXvel_type;
  _rotXvel_type rotXvel;

   typedef double _rotYvel_type;
  _rotYvel_type rotYvel;

   typedef double _rotZvel_type;
  _rotZvel_type rotZvel;




  typedef boost::shared_ptr< ::ucl_drone::Pose3D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucl_drone::Pose3D_<ContainerAllocator> const> ConstPtr;

}; // struct Pose3D_

typedef ::ucl_drone::Pose3D_<std::allocator<void> > Pose3D;

typedef boost::shared_ptr< ::ucl_drone::Pose3D > Pose3DPtr;
typedef boost::shared_ptr< ::ucl_drone::Pose3D const> Pose3DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucl_drone::Pose3D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucl_drone::Pose3D_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ucl_drone

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'ucl_drone': ['/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/laboinmastudent/Bureau/Drone_thesis_2017/Additional_packages/src/ardrone_autonomy/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::Pose3D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::Pose3D_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::Pose3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::Pose3D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::Pose3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::Pose3D_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucl_drone::Pose3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "61cfa76c32b638d5304ee331d87c6c2b";
  }

  static const char* value(const ::ucl_drone::Pose3D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x61cfa76c32b638d5ULL;
  static const uint64_t static_value2 = 0x304ee331d87c6c2bULL;
};

template<class ContainerAllocator>
struct DataType< ::ucl_drone::Pose3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucl_drone/Pose3D";
  }

  static const char* value(const ::ucl_drone::Pose3D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucl_drone::Pose3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
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
";
  }

  static const char* value(const ::ucl_drone::Pose3D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucl_drone::Pose3D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.rotX);
      stream.next(m.rotY);
      stream.next(m.rotZ);
      stream.next(m.xvel);
      stream.next(m.yvel);
      stream.next(m.zvel);
      stream.next(m.rotXvel);
      stream.next(m.rotYvel);
      stream.next(m.rotZvel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pose3D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucl_drone::Pose3D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucl_drone::Pose3D_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "rotX: ";
    Printer<double>::stream(s, indent + "  ", v.rotX);
    s << indent << "rotY: ";
    Printer<double>::stream(s, indent + "  ", v.rotY);
    s << indent << "rotZ: ";
    Printer<double>::stream(s, indent + "  ", v.rotZ);
    s << indent << "xvel: ";
    Printer<double>::stream(s, indent + "  ", v.xvel);
    s << indent << "yvel: ";
    Printer<double>::stream(s, indent + "  ", v.yvel);
    s << indent << "zvel: ";
    Printer<double>::stream(s, indent + "  ", v.zvel);
    s << indent << "rotXvel: ";
    Printer<double>::stream(s, indent + "  ", v.rotXvel);
    s << indent << "rotYvel: ";
    Printer<double>::stream(s, indent + "  ", v.rotYvel);
    s << indent << "rotZvel: ";
    Printer<double>::stream(s, indent + "  ", v.rotZvel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCL_DRONE_MESSAGE_POSE3D_H
