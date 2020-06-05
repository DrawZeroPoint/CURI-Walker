// Generated by gencpp from file cruiser_msgs/cruiserJointActionResponse.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISERJOINTACTIONRESPONSE_H
#define CRUISER_MSGS_MESSAGE_CRUISERJOINTACTIONRESPONSE_H


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
struct cruiserJointActionResponse_
{
  typedef cruiserJointActionResponse_<ContainerAllocator> Type;

  cruiserJointActionResponse_()
    : result()  {
    }
  cruiserJointActionResponse_(const ContainerAllocator& _alloc)
    : result(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct cruiserJointActionResponse_

typedef ::cruiser_msgs::cruiserJointActionResponse_<std::allocator<void> > cruiserJointActionResponse;

typedef boost::shared_ptr< ::cruiser_msgs::cruiserJointActionResponse > cruiserJointActionResponsePtr;
typedef boost::shared_ptr< ::cruiser_msgs::cruiserJointActionResponse const> cruiserJointActionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c22f2a1ed8654a0b365f1bb3f7ff2c0f";
  }

  static const char* value(const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc22f2a1ed8654a0bULL;
  static const uint64_t static_value2 = 0x365f1bb3f7ff2c0fULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/cruiserJointActionResponse";
  }

  static const char* value(const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"string result\n"
"\n"
"\n"
;
  }

  static const char* value(const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cruiserJointActionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::cruiserJointActionResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISERJOINTACTIONRESPONSE_H
