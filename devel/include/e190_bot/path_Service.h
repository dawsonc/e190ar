// Generated by gencpp from file e190_bot/path_Service.msg
// DO NOT EDIT!


#ifndef E190_BOT_MESSAGE_PATH_SERVICE_H
#define E190_BOT_MESSAGE_PATH_SERVICE_H

#include <ros/service_traits.h>


#include <e190_bot/path_ServiceRequest.h>
#include <e190_bot/path_ServiceResponse.h>


namespace e190_bot
{

struct path_Service
{

typedef path_ServiceRequest Request;
typedef path_ServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct path_Service
} // namespace e190_bot


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::e190_bot::path_Service > {
  static const char* value()
  {
    return "9d2f833ec7c0e39cfbf37318393587cd";
  }

  static const char* value(const ::e190_bot::path_Service&) { return value(); }
};

template<>
struct DataType< ::e190_bot::path_Service > {
  static const char* value()
  {
    return "e190_bot/path_Service";
  }

  static const char* value(const ::e190_bot::path_Service&) { return value(); }
};


// service_traits::MD5Sum< ::e190_bot::path_ServiceRequest> should match 
// service_traits::MD5Sum< ::e190_bot::path_Service > 
template<>
struct MD5Sum< ::e190_bot::path_ServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::e190_bot::path_Service >::value();
  }
  static const char* value(const ::e190_bot::path_ServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::e190_bot::path_ServiceRequest> should match 
// service_traits::DataType< ::e190_bot::path_Service > 
template<>
struct DataType< ::e190_bot::path_ServiceRequest>
{
  static const char* value()
  {
    return DataType< ::e190_bot::path_Service >::value();
  }
  static const char* value(const ::e190_bot::path_ServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::e190_bot::path_ServiceResponse> should match 
// service_traits::MD5Sum< ::e190_bot::path_Service > 
template<>
struct MD5Sum< ::e190_bot::path_ServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::e190_bot::path_Service >::value();
  }
  static const char* value(const ::e190_bot::path_ServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::e190_bot::path_ServiceResponse> should match 
// service_traits::DataType< ::e190_bot::path_Service > 
template<>
struct DataType< ::e190_bot::path_ServiceResponse>
{
  static const char* value()
  {
    return DataType< ::e190_bot::path_Service >::value();
  }
  static const char* value(const ::e190_bot::path_ServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // E190_BOT_MESSAGE_PATH_SERVICE_H
