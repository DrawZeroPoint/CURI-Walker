// Generated by gencpp from file ces_task/TaskLegCtrlRequest.msg
// DO NOT EDIT!


#ifndef CES_TASK_MESSAGE_TASKLEGCTRLREQUEST_H
#define CES_TASK_MESSAGE_TASKLEGCTRLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ces_task
{
template <class ContainerAllocator>
struct TaskLegCtrlRequest_
{
  typedef TaskLegCtrlRequest_<ContainerAllocator> Type;

  TaskLegCtrlRequest_()
    : task_id()
    , demander()
    , executor()
    , cmd()  {
    }
  TaskLegCtrlRequest_(const ContainerAllocator& _alloc)
    : task_id(_alloc)
    , demander(_alloc)
    , executor(_alloc)
    , cmd(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _task_id_type;
  _task_id_type task_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _demander_type;
  _demander_type demander;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _executor_type;
  _executor_type executor;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cmd_type;
  _cmd_type cmd;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CMD_START)
  #undef CMD_START
#endif
#if defined(_WIN32) && defined(CMD_STOP)
  #undef CMD_STOP
#endif


  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  CMD_START;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  CMD_STOP;

  typedef boost::shared_ptr< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TaskLegCtrlRequest_

typedef ::ces_task::TaskLegCtrlRequest_<std::allocator<void> > TaskLegCtrlRequest;

typedef boost::shared_ptr< ::ces_task::TaskLegCtrlRequest > TaskLegCtrlRequestPtr;
typedef boost::shared_ptr< ::ces_task::TaskLegCtrlRequest const> TaskLegCtrlRequestConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskLegCtrlRequest_<ContainerAllocator>::CMD_START =
        
          "start"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskLegCtrlRequest_<ContainerAllocator>::CMD_STOP =
        
          "stop"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator1> & lhs, const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator2> & rhs)
{
  return lhs.task_id == rhs.task_id &&
    lhs.demander == rhs.demander &&
    lhs.executor == rhs.executor &&
    lhs.cmd == rhs.cmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator1> & lhs, const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ces_task

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c31edd7c6e13e952878e69949e90ae23";
  }

  static const char* value(const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc31edd7c6e13e952ULL;
  static const uint64_t static_value2 = 0x878e69949e90ae23ULL;
};

template<class ContainerAllocator>
struct DataType< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ces_task/TaskLegCtrlRequest";
  }

  static const char* value(const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"string task_id\n"
"\n"
"\n"
"string demander\n"
"\n"
"\n"
"string executor\n"
"\n"
"\n"
"string CMD_START=start\n"
"string CMD_STOP=stop\n"
"string cmd\n"
"\n"
;
  }

  static const char* value(const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.task_id);
      stream.next(m.demander);
      stream.next(m.executor);
      stream.next(m.cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TaskLegCtrlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ces_task::TaskLegCtrlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ces_task::TaskLegCtrlRequest_<ContainerAllocator>& v)
  {
    s << indent << "task_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.task_id);
    s << indent << "demander: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.demander);
    s << indent << "executor: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.executor);
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CES_TASK_MESSAGE_TASKLEGCTRLREQUEST_H
