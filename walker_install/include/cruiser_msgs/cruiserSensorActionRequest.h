// Generated by gencpp from file cruiser_msgs/cruiserSensorActionRequest.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISERSENSORACTIONREQUEST_H
#define CRUISER_MSGS_MESSAGE_CRUISERSENSORACTIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cruiser_msgs
{
template <class ContainerAllocator>
struct cruiserSensorActionRequest_
{
  typedef cruiserSensorActionRequest_<ContainerAllocator> Type;

  cruiserSensorActionRequest_()
    : ultrasound_freq(0)
    , wall_ir_freq(0)
    , charge_ir_freq(0)  {
    }
  cruiserSensorActionRequest_(const ContainerAllocator& _alloc)
    : ultrasound_freq(0)
    , wall_ir_freq(0)
    , charge_ir_freq(0)  {
  (void)_alloc;
    }



   typedef uint32_t _ultrasound_freq_type;
  _ultrasound_freq_type ultrasound_freq;

   typedef uint32_t _wall_ir_freq_type;
  _wall_ir_freq_type wall_ir_freq;

   typedef uint32_t _charge_ir_freq_type;
  _charge_ir_freq_type charge_ir_freq;





  typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct cruiserSensorActionRequest_

typedef ::cruiser_msgs::cruiserSensorActionRequest_<std::allocator<void> > cruiserSensorActionRequest;

typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionRequest > cruiserSensorActionRequestPtr;
typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionRequest const> cruiserSensorActionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator2> & rhs)
{
  return lhs.ultrasound_freq == rhs.ultrasound_freq &&
    lhs.wall_ir_freq == rhs.wall_ir_freq &&
    lhs.charge_ir_freq == rhs.charge_ir_freq;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "42deacc6f05bc5700ad089daf0418ae6";
  }

  static const char* value(const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x42deacc6f05bc570ULL;
  static const uint64_t static_value2 = 0x0ad089daf0418ae6ULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/cruiserSensorActionRequest";
  }

  static const char* value(const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 ultrasound_freq\n"
"uint32 wall_ir_freq\n"
"uint32 charge_ir_freq\n"
;
  }

  static const char* value(const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ultrasound_freq);
      stream.next(m.wall_ir_freq);
      stream.next(m.charge_ir_freq);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cruiserSensorActionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::cruiserSensorActionRequest_<ContainerAllocator>& v)
  {
    s << indent << "ultrasound_freq: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.ultrasound_freq);
    s << indent << "wall_ir_freq: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.wall_ir_freq);
    s << indent << "charge_ir_freq: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.charge_ir_freq);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISERSENSORACTIONREQUEST_H
