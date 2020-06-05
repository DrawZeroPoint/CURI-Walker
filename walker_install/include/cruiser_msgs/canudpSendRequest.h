// Generated by gencpp from file cruiser_msgs/canudpSendRequest.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CANUDPSENDREQUEST_H
#define CRUISER_MSGS_MESSAGE_CANUDPSENDREQUEST_H


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
struct canudpSendRequest_
{
  typedef canudpSendRequest_<ContainerAllocator> Type;

  canudpSendRequest_()
    : priority(0)
    , packetdata()  {
    }
  canudpSendRequest_(const ContainerAllocator& _alloc)
    : priority(0)
    , packetdata(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _priority_type;
  _priority_type priority;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _packetdata_type;
  _packetdata_type packetdata;





  typedef boost::shared_ptr< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> const> ConstPtr;

}; // struct canudpSendRequest_

typedef ::cruiser_msgs::canudpSendRequest_<std::allocator<void> > canudpSendRequest;

typedef boost::shared_ptr< ::cruiser_msgs::canudpSendRequest > canudpSendRequestPtr;
typedef boost::shared_ptr< ::cruiser_msgs::canudpSendRequest const> canudpSendRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator1> & lhs, const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator2> & rhs)
{
  return lhs.priority == rhs.priority &&
    lhs.packetdata == rhs.packetdata;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator1> & lhs, const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c074140e21c6ef6ba56748714f78878f";
  }

  static const char* value(const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc074140e21c6ef6bULL;
  static const uint64_t static_value2 = 0xa56748714f78878fULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/canudpSendRequest";
  }

  static const char* value(const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 priority\n"
"string packetdata\n"
;
  }

  static const char* value(const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.priority);
      stream.next(m.packetdata);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct canudpSendRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::canudpSendRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::canudpSendRequest_<ContainerAllocator>& v)
  {
    s << indent << "priority: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.priority);
    s << indent << "packetdata: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.packetdata);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CANUDPSENDREQUEST_H
