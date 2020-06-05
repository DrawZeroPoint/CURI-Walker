// Generated by gencpp from file cruiser_msgs/UwbAnchor.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_UWBANCHOR_H
#define CRUISER_MSGS_MESSAGE_UWBANCHOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace cruiser_msgs
{
template <class ContainerAllocator>
struct UwbAnchor_
{
  typedef UwbAnchor_<ContainerAllocator> Type;

  UwbAnchor_()
    : anchors()  {
    }
  UwbAnchor_(const ContainerAllocator& _alloc)
    : anchors(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _anchors_type;
  _anchors_type anchors;





  typedef boost::shared_ptr< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> const> ConstPtr;

}; // struct UwbAnchor_

typedef ::cruiser_msgs::UwbAnchor_<std::allocator<void> > UwbAnchor;

typedef boost::shared_ptr< ::cruiser_msgs::UwbAnchor > UwbAnchorPtr;
typedef boost::shared_ptr< ::cruiser_msgs::UwbAnchor const> UwbAnchorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::UwbAnchor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::UwbAnchor_<ContainerAllocator1> & lhs, const ::cruiser_msgs::UwbAnchor_<ContainerAllocator2> & rhs)
{
  return lhs.anchors == rhs.anchors;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::UwbAnchor_<ContainerAllocator1> & lhs, const ::cruiser_msgs::UwbAnchor_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bb69d45428817d4189c9bb41446d70fb";
  }

  static const char* value(const ::cruiser_msgs::UwbAnchor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbb69d45428817d41ULL;
  static const uint64_t static_value2 = 0x89c9bb41446d70fbULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/UwbAnchor";
  }

  static const char* value(const ::cruiser_msgs::UwbAnchor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PoseStamped[] anchors\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::cruiser_msgs::UwbAnchor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.anchors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UwbAnchor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::UwbAnchor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::UwbAnchor_<ContainerAllocator>& v)
  {
    s << indent << "anchors[]" << std::endl;
    for (size_t i = 0; i < v.anchors.size(); ++i)
    {
      s << indent << "  anchors[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.anchors[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_UWBANCHOR_H
