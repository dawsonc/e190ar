// Generated by gencpp from file e190_bot/path_ServiceResponse.msg
// DO NOT EDIT!


#ifndef E190_BOT_MESSAGE_PATH_SERVICERESPONSE_H
#define E190_BOT_MESSAGE_PATH_SERVICERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace e190_bot
{
template <class ContainerAllocator>
struct path_ServiceResponse_
{
  typedef path_ServiceResponse_<ContainerAllocator> Type;

  path_ServiceResponse_()
    : next()  {
    }
  path_ServiceResponse_(const ContainerAllocator& _alloc)
    : next(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _next_type;
  _next_type next;





  typedef boost::shared_ptr< ::e190_bot::path_ServiceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::e190_bot::path_ServiceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct path_ServiceResponse_

typedef ::e190_bot::path_ServiceResponse_<std::allocator<void> > path_ServiceResponse;

typedef boost::shared_ptr< ::e190_bot::path_ServiceResponse > path_ServiceResponsePtr;
typedef boost::shared_ptr< ::e190_bot::path_ServiceResponse const> path_ServiceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::e190_bot::path_ServiceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace e190_bot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'e190_bot': ['/home/peter/190_ws/src/e190_bot/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::e190_bot::path_ServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::e190_bot::path_ServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::e190_bot::path_ServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "06c87fd674ca2e488185f7a8035cf5fd";
  }

  static const char* value(const ::e190_bot::path_ServiceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x06c87fd674ca2e48ULL;
  static const uint64_t static_value2 = 0x8185f7a8035cf5fdULL;
};

template<class ContainerAllocator>
struct DataType< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e190_bot/path_ServiceResponse";
  }

  static const char* value(const ::e190_bot::path_ServiceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PoseStamped next\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::e190_bot::path_ServiceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.next);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct path_ServiceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::e190_bot::path_ServiceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::e190_bot::path_ServiceResponse_<ContainerAllocator>& v)
  {
    s << indent << "next: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.next);
  }
};

} // namespace message_operations
} // namespace ros

#endif // E190_BOT_MESSAGE_PATH_SERVICERESPONSE_H
