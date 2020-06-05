// Generated by gencpp from file ubt_core_msgs/HeadState.msg
// DO NOT EDIT!


#ifndef UBT_CORE_MSGS_MESSAGE_HEADSTATE_H
#define UBT_CORE_MSGS_MESSAGE_HEADSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ubt_core_msgs
{
template <class ContainerAllocator>
struct HeadState_
{
  typedef HeadState_<ContainerAllocator> Type;

  HeadState_()
    : pan(0.0)
    , isTurning(false)
    , isNodding(false)
    , isPanEnabled(false)  {
    }
  HeadState_(const ContainerAllocator& _alloc)
    : pan(0.0)
    , isTurning(false)
    , isNodding(false)
    , isPanEnabled(false)  {
  (void)_alloc;
    }



   typedef float _pan_type;
  _pan_type pan;

   typedef uint8_t _isTurning_type;
  _isTurning_type isTurning;

   typedef uint8_t _isNodding_type;
  _isNodding_type isNodding;

   typedef uint8_t _isPanEnabled_type;
  _isPanEnabled_type isPanEnabled;





  typedef boost::shared_ptr< ::ubt_core_msgs::HeadState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ubt_core_msgs::HeadState_<ContainerAllocator> const> ConstPtr;

}; // struct HeadState_

typedef ::ubt_core_msgs::HeadState_<std::allocator<void> > HeadState;

typedef boost::shared_ptr< ::ubt_core_msgs::HeadState > HeadStatePtr;
typedef boost::shared_ptr< ::ubt_core_msgs::HeadState const> HeadStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ubt_core_msgs::HeadState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ubt_core_msgs::HeadState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ubt_core_msgs::HeadState_<ContainerAllocator1> & lhs, const ::ubt_core_msgs::HeadState_<ContainerAllocator2> & rhs)
{
  return lhs.pan == rhs.pan &&
    lhs.isTurning == rhs.isTurning &&
    lhs.isNodding == rhs.isNodding &&
    lhs.isPanEnabled == rhs.isPanEnabled;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ubt_core_msgs::HeadState_<ContainerAllocator1> & lhs, const ::ubt_core_msgs::HeadState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ubt_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ubt_core_msgs::HeadState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ubt_core_msgs::HeadState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ubt_core_msgs::HeadState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "71c43b264307205358e7e49be5601348";
  }

  static const char* value(const ::ubt_core_msgs::HeadState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x71c43b2643072053ULL;
  static const uint64_t static_value2 = 0x58e7e49be5601348ULL;
};

template<class ContainerAllocator>
struct DataType< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ubt_core_msgs/HeadState";
  }

  static const char* value(const ::ubt_core_msgs::HeadState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pan\n"
"bool isTurning\n"
"bool isNodding\n"
"bool isPanEnabled\n"
;
  }

  static const char* value(const ::ubt_core_msgs::HeadState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pan);
      stream.next(m.isTurning);
      stream.next(m.isNodding);
      stream.next(m.isPanEnabled);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HeadState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ubt_core_msgs::HeadState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ubt_core_msgs::HeadState_<ContainerAllocator>& v)
  {
    s << indent << "pan: ";
    Printer<float>::stream(s, indent + "  ", v.pan);
    s << indent << "isTurning: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isTurning);
    s << indent << "isNodding: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isNodding);
    s << indent << "isPanEnabled: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isPanEnabled);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBT_CORE_MSGS_MESSAGE_HEADSTATE_H
