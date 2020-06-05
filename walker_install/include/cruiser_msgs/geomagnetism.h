// Generated by gencpp from file cruiser_msgs/geomagnetism.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_GEOMAGNETISM_H
#define CRUISER_MSGS_MESSAGE_GEOMAGNETISM_H


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
struct geomagnetism_
{
  typedef geomagnetism_<ContainerAllocator> Type;

  geomagnetism_()
    : gyro_x(0)
    , gyro_y(0)
    , gyro_z(0)
    , acc_x(0)
    , acc_y(0)
    , acc_z(0)
    , magnetic_x(0)
    , magnetic_y(0)
    , magnetic_z(0)
    , euler_pitch(0)
    , euler_roll(0)
    , euler_yaw(0)
    , trigger(false)  {
    }
  geomagnetism_(const ContainerAllocator& _alloc)
    : gyro_x(0)
    , gyro_y(0)
    , gyro_z(0)
    , acc_x(0)
    , acc_y(0)
    , acc_z(0)
    , magnetic_x(0)
    , magnetic_y(0)
    , magnetic_z(0)
    , euler_pitch(0)
    , euler_roll(0)
    , euler_yaw(0)
    , trigger(false)  {
  (void)_alloc;
    }



   typedef int32_t _gyro_x_type;
  _gyro_x_type gyro_x;

   typedef int32_t _gyro_y_type;
  _gyro_y_type gyro_y;

   typedef int32_t _gyro_z_type;
  _gyro_z_type gyro_z;

   typedef int32_t _acc_x_type;
  _acc_x_type acc_x;

   typedef int32_t _acc_y_type;
  _acc_y_type acc_y;

   typedef int32_t _acc_z_type;
  _acc_z_type acc_z;

   typedef int32_t _magnetic_x_type;
  _magnetic_x_type magnetic_x;

   typedef int32_t _magnetic_y_type;
  _magnetic_y_type magnetic_y;

   typedef int32_t _magnetic_z_type;
  _magnetic_z_type magnetic_z;

   typedef int32_t _euler_pitch_type;
  _euler_pitch_type euler_pitch;

   typedef int32_t _euler_roll_type;
  _euler_roll_type euler_roll;

   typedef int32_t _euler_yaw_type;
  _euler_yaw_type euler_yaw;

   typedef uint8_t _trigger_type;
  _trigger_type trigger;





  typedef boost::shared_ptr< ::cruiser_msgs::geomagnetism_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::geomagnetism_<ContainerAllocator> const> ConstPtr;

}; // struct geomagnetism_

typedef ::cruiser_msgs::geomagnetism_<std::allocator<void> > geomagnetism;

typedef boost::shared_ptr< ::cruiser_msgs::geomagnetism > geomagnetismPtr;
typedef boost::shared_ptr< ::cruiser_msgs::geomagnetism const> geomagnetismConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::geomagnetism_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::geomagnetism_<ContainerAllocator1> & lhs, const ::cruiser_msgs::geomagnetism_<ContainerAllocator2> & rhs)
{
  return lhs.gyro_x == rhs.gyro_x &&
    lhs.gyro_y == rhs.gyro_y &&
    lhs.gyro_z == rhs.gyro_z &&
    lhs.acc_x == rhs.acc_x &&
    lhs.acc_y == rhs.acc_y &&
    lhs.acc_z == rhs.acc_z &&
    lhs.magnetic_x == rhs.magnetic_x &&
    lhs.magnetic_y == rhs.magnetic_y &&
    lhs.magnetic_z == rhs.magnetic_z &&
    lhs.euler_pitch == rhs.euler_pitch &&
    lhs.euler_roll == rhs.euler_roll &&
    lhs.euler_yaw == rhs.euler_yaw &&
    lhs.trigger == rhs.trigger;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::geomagnetism_<ContainerAllocator1> & lhs, const ::cruiser_msgs::geomagnetism_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::geomagnetism_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::geomagnetism_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::geomagnetism_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a11ec4346fd95eab51cf9d95e5a6dc16";
  }

  static const char* value(const ::cruiser_msgs::geomagnetism_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa11ec4346fd95eabULL;
  static const uint64_t static_value2 = 0x51cf9d95e5a6dc16ULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/geomagnetism";
  }

  static const char* value(const ::cruiser_msgs::geomagnetism_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 gyro_x\n"
"int32 gyro_y\n"
"int32 gyro_z\n"
"int32 acc_x\n"
"int32 acc_y\n"
"int32 acc_z\n"
"int32 magnetic_x\n"
"int32 magnetic_y\n"
"int32 magnetic_z\n"
"int32 euler_pitch\n"
"int32 euler_roll\n"
"int32 euler_yaw\n"
"bool trigger\n"
"\n"
"\n"
"\n"
"\n"
;
  }

  static const char* value(const ::cruiser_msgs::geomagnetism_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gyro_x);
      stream.next(m.gyro_y);
      stream.next(m.gyro_z);
      stream.next(m.acc_x);
      stream.next(m.acc_y);
      stream.next(m.acc_z);
      stream.next(m.magnetic_x);
      stream.next(m.magnetic_y);
      stream.next(m.magnetic_z);
      stream.next(m.euler_pitch);
      stream.next(m.euler_roll);
      stream.next(m.euler_yaw);
      stream.next(m.trigger);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct geomagnetism_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::geomagnetism_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::geomagnetism_<ContainerAllocator>& v)
  {
    s << indent << "gyro_x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gyro_x);
    s << indent << "gyro_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gyro_y);
    s << indent << "gyro_z: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gyro_z);
    s << indent << "acc_x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acc_x);
    s << indent << "acc_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acc_y);
    s << indent << "acc_z: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acc_z);
    s << indent << "magnetic_x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.magnetic_x);
    s << indent << "magnetic_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.magnetic_y);
    s << indent << "magnetic_z: ";
    Printer<int32_t>::stream(s, indent + "  ", v.magnetic_z);
    s << indent << "euler_pitch: ";
    Printer<int32_t>::stream(s, indent + "  ", v.euler_pitch);
    s << indent << "euler_roll: ";
    Printer<int32_t>::stream(s, indent + "  ", v.euler_roll);
    s << indent << "euler_yaw: ";
    Printer<int32_t>::stream(s, indent + "  ", v.euler_yaw);
    s << indent << "trigger: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.trigger);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_GEOMAGNETISM_H
