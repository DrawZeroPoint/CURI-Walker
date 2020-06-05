// Generated by gencpp from file servo_ctrl/SendFtAtCmd.msg
// DO NOT EDIT!


#ifndef SERVO_CTRL_MESSAGE_SENDFTATCMD_H
#define SERVO_CTRL_MESSAGE_SENDFTATCMD_H

#include <ros/service_traits.h>


#include <servo_ctrl/SendFtAtCmdRequest.h>
#include <servo_ctrl/SendFtAtCmdResponse.h>


namespace servo_ctrl
{

struct SendFtAtCmd
{

typedef SendFtAtCmdRequest Request;
typedef SendFtAtCmdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SendFtAtCmd
} // namespace servo_ctrl


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::servo_ctrl::SendFtAtCmd > {
  static const char* value()
  {
    return "6047a75169370a000cccb1c1e8fefc47";
  }

  static const char* value(const ::servo_ctrl::SendFtAtCmd&) { return value(); }
};

template<>
struct DataType< ::servo_ctrl::SendFtAtCmd > {
  static const char* value()
  {
    return "servo_ctrl/SendFtAtCmd";
  }

  static const char* value(const ::servo_ctrl::SendFtAtCmd&) { return value(); }
};


// service_traits::MD5Sum< ::servo_ctrl::SendFtAtCmdRequest> should match
// service_traits::MD5Sum< ::servo_ctrl::SendFtAtCmd >
template<>
struct MD5Sum< ::servo_ctrl::SendFtAtCmdRequest>
{
  static const char* value()
  {
    return MD5Sum< ::servo_ctrl::SendFtAtCmd >::value();
  }
  static const char* value(const ::servo_ctrl::SendFtAtCmdRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::servo_ctrl::SendFtAtCmdRequest> should match
// service_traits::DataType< ::servo_ctrl::SendFtAtCmd >
template<>
struct DataType< ::servo_ctrl::SendFtAtCmdRequest>
{
  static const char* value()
  {
    return DataType< ::servo_ctrl::SendFtAtCmd >::value();
  }
  static const char* value(const ::servo_ctrl::SendFtAtCmdRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::servo_ctrl::SendFtAtCmdResponse> should match
// service_traits::MD5Sum< ::servo_ctrl::SendFtAtCmd >
template<>
struct MD5Sum< ::servo_ctrl::SendFtAtCmdResponse>
{
  static const char* value()
  {
    return MD5Sum< ::servo_ctrl::SendFtAtCmd >::value();
  }
  static const char* value(const ::servo_ctrl::SendFtAtCmdResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::servo_ctrl::SendFtAtCmdResponse> should match
// service_traits::DataType< ::servo_ctrl::SendFtAtCmd >
template<>
struct DataType< ::servo_ctrl::SendFtAtCmdResponse>
{
  static const char* value()
  {
    return DataType< ::servo_ctrl::SendFtAtCmd >::value();
  }
  static const char* value(const ::servo_ctrl::SendFtAtCmdResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SERVO_CTRL_MESSAGE_SENDFTATCMD_H
