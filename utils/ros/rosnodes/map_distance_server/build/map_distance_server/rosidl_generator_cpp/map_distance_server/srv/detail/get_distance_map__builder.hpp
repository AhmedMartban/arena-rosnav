// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from map_distance_server:srv/GetDistanceMap.idl
// generated code does not contain a copyright notice

#ifndef MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__BUILDER_HPP_
#define MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "map_distance_server/srv/detail/get_distance_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace map_distance_server
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::map_distance_server::srv::GetDistanceMap_Request>()
{
  return ::map_distance_server::srv::GetDistanceMap_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace map_distance_server


namespace map_distance_server
{

namespace srv
{

namespace builder
{

class Init_GetDistanceMap_Response_data
{
public:
  explicit Init_GetDistanceMap_Response_data(::map_distance_server::srv::GetDistanceMap_Response & msg)
  : msg_(msg)
  {}
  ::map_distance_server::srv::GetDistanceMap_Response data(::map_distance_server::srv::GetDistanceMap_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::map_distance_server::srv::GetDistanceMap_Response msg_;
};

class Init_GetDistanceMap_Response_info
{
public:
  explicit Init_GetDistanceMap_Response_info(::map_distance_server::srv::GetDistanceMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetDistanceMap_Response_data info(::map_distance_server::srv::GetDistanceMap_Response::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetDistanceMap_Response_data(msg_);
  }

private:
  ::map_distance_server::srv::GetDistanceMap_Response msg_;
};

class Init_GetDistanceMap_Response_header
{
public:
  Init_GetDistanceMap_Response_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDistanceMap_Response_info header(::map_distance_server::srv::GetDistanceMap_Response::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GetDistanceMap_Response_info(msg_);
  }

private:
  ::map_distance_server::srv::GetDistanceMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::map_distance_server::srv::GetDistanceMap_Response>()
{
  return map_distance_server::srv::builder::Init_GetDistanceMap_Response_header();
}

}  // namespace map_distance_server

#endif  // MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__BUILDER_HPP_
