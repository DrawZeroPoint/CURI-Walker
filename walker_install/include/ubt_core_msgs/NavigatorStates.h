// Generated by gencpp from file ubt_core_msgs/NavigatorStates.msg
// DO NOT EDIT!


#ifndef UBT_CORE_MSGS_MESSAGE_NAVIGATORSTATES_H
#define UBT_CORE_MSGS_MESSAGE_NAVIGATORSTATES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ubt_core_msgs/NavigatorState.h>

namespace ubt_core_msgs
{
template <class ContainerAllocator>
struct NavigatorStates_
{
  typedef NavigatorStates_<ContainerAllocator> Type;

  NavigatorStates_()
    : names()
    , states()  {
    }
  NavigatorStates_(const ContainerAllocator& _alloc)
    : names(_alloc)
    , states(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _names_type;
  _names_type names;

   typedef std::vector< ::ubt_core_msgs::NavigatorState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ubt_core_msgs::NavigatorState_<ContainerAllocator> >::other >  _states_type;
  _states_type states;





  typedef boost::shared_ptr< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> const> ConstPtr;

}; // struct NavigatorStates_

typedef ::ubt_core_msgs::NavigatorStates_<std::allocator<void> > NavigatorStates;

typedef boost::shared_ptr< ::ubt_core_msgs::NavigatorStates > NavigatorStatesPtr;
typedef boost::shared_ptr< ::ubt_core_msgs::NavigatorStates const> NavigatorStatesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator1> & lhs, const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator2> & rhs)
{
  return lhs.names == rhs.names &&
    lhs.states == rhs.states;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator1> & lhs, const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ubt_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2c2eeb02fbbaa6f1ab6c680887f2db78";
  }

  static const char* value(const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2c2eeb02fbbaa6f1ULL;
  static const uint64_t static_value2 = 0xab6c680887f2db78ULL;
};

template<class ContainerAllocator>
struct DataType< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ubt_core_msgs/NavigatorStates";
  }

  static const char* value(const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# used when publishing multiple navigators\n"
"string[]         names\n"
"NavigatorState[] states\n"
"\n"
"================================================================================\n"
"MSG: ubt_core_msgs/NavigatorState\n"
"# buttons\n"
"string[] button_names\n"
"bool[] buttons\n"
"\n"
"# wheel position\n"
"uint8   wheel\n"
"\n"
"# true if the light is on, false if not\n"
"# lights map to button names\n"
"string[] light_names\n"
"bool[] lights\n"
;
  }

  static const char* value(const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.names);
      stream.next(m.states);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigatorStates_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ubt_core_msgs::NavigatorStates_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ubt_core_msgs::NavigatorStates_<ContainerAllocator>& v)
  {
    s << indent << "names[]" << std::endl;
    for (size_t i = 0; i < v.names.size(); ++i)
    {
      s << indent << "  names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.names[i]);
    }
    s << indent << "states[]" << std::endl;
    for (size_t i = 0; i < v.states.size(); ++i)
    {
      s << indent << "  states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ubt_core_msgs::NavigatorState_<ContainerAllocator> >::stream(s, indent + "    ", v.states[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBT_CORE_MSGS_MESSAGE_NAVIGATORSTATES_H
