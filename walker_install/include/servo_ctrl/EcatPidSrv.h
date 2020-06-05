// Generated by gencpp from file servo_ctrl/EcatPidSrv.msg
// DO NOT EDIT!


#ifndef SERVO_CTRL_MESSAGE_ECATPIDSRV_H
#define SERVO_CTRL_MESSAGE_ECATPIDSRV_H

#include <ros/service_traits.h>


#include <servo_ctrl/EcatPidSrvRequest.h>
#include <servo_ctrl/EcatPidSrvResponse.h>


namespace servo_ctrl
{

struct EcatPidSrv
{

typedef EcatPidSrvRequest Request;
typedef EcatPidSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EcatPidSrv
} // namespace servo_ctrl


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::servo_ctrl::EcatPidSrv > {
  static const char* value()
  {
    return "7b9594603cb192f9d9c16b8685314164";
  }

  static const char* value(const ::servo_ctrl::EcatPidSrv&) { return value(); }
};

template<>
struct DataType< ::servo_ctrl::EcatPidSrv > {
  static const char* value()
  {
    return "servo_ctrl/EcatPidSrv";
  }

  static const char* value(const ::servo_ctrl::EcatPidSrv&) { return value(); }
};


// service_traits::MD5Sum< ::servo_ctrl::EcatPidSrvRequest> should match
// service_traits::MD5Sum< ::servo_ctrl::EcatPidSrv >
template<>
struct MD5Sum< ::servo_ctrl::EcatPidSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::servo_ctrl::EcatPidSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatPidSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::servo_ctrl::EcatPidSrvRequest> should match
// service_traits::DataType< ::servo_ctrl::EcatPidSrv >
template<>
struct DataType< ::servo_ctrl::EcatPidSrvRequest>
{
  static const char* value()
  {
    return DataType< ::servo_ctrl::EcatPidSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatPidSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::servo_ctrl::EcatPidSrvResponse> should match
// service_traits::MD5Sum< ::servo_ctrl::EcatPidSrv >
template<>
struct MD5Sum< ::servo_ctrl::EcatPidSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::servo_ctrl::EcatPidSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatPidSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::servo_ctrl::EcatPidSrvResponse> should match
// service_traits::DataType< ::servo_ctrl::EcatPidSrv >
template<>
struct DataType< ::servo_ctrl::EcatPidSrvResponse>
{
  static const char* value()
  {
    return DataType< ::servo_ctrl::EcatPidSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatPidSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SERVO_CTRL_MESSAGE_ECATPIDSRV_H
