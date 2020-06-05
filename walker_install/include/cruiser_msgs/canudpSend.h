// Generated by gencpp from file cruiser_msgs/canudpSend.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CANUDPSEND_H
#define CRUISER_MSGS_MESSAGE_CANUDPSEND_H

#include <ros/service_traits.h>


#include <cruiser_msgs/canudpSendRequest.h>
#include <cruiser_msgs/canudpSendResponse.h>


namespace cruiser_msgs
{

struct canudpSend
{

typedef canudpSendRequest Request;
typedef canudpSendResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct canudpSend
} // namespace cruiser_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cruiser_msgs::canudpSend > {
  static const char* value()
  {
    return "5b1b6b129155e5a4f811f32ad75d66ed";
  }

  static const char* value(const ::cruiser_msgs::canudpSend&) { return value(); }
};

template<>
struct DataType< ::cruiser_msgs::canudpSend > {
  static const char* value()
  {
    return "cruiser_msgs/canudpSend";
  }

  static const char* value(const ::cruiser_msgs::canudpSend&) { return value(); }
};


// service_traits::MD5Sum< ::cruiser_msgs::canudpSendRequest> should match
// service_traits::MD5Sum< ::cruiser_msgs::canudpSend >
template<>
struct MD5Sum< ::cruiser_msgs::canudpSendRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cruiser_msgs::canudpSend >::value();
  }
  static const char* value(const ::cruiser_msgs::canudpSendRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cruiser_msgs::canudpSendRequest> should match
// service_traits::DataType< ::cruiser_msgs::canudpSend >
template<>
struct DataType< ::cruiser_msgs::canudpSendRequest>
{
  static const char* value()
  {
    return DataType< ::cruiser_msgs::canudpSend >::value();
  }
  static const char* value(const ::cruiser_msgs::canudpSendRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cruiser_msgs::canudpSendResponse> should match
// service_traits::MD5Sum< ::cruiser_msgs::canudpSend >
template<>
struct MD5Sum< ::cruiser_msgs::canudpSendResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cruiser_msgs::canudpSend >::value();
  }
  static const char* value(const ::cruiser_msgs::canudpSendResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cruiser_msgs::canudpSendResponse> should match
// service_traits::DataType< ::cruiser_msgs::canudpSend >
template<>
struct DataType< ::cruiser_msgs::canudpSendResponse>
{
  static const char* value()
  {
    return DataType< ::cruiser_msgs::canudpSend >::value();
  }
  static const char* value(const ::cruiser_msgs::canudpSendResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CANUDPSEND_H
