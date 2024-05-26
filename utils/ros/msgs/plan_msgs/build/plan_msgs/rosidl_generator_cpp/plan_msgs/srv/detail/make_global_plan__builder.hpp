// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:srv/MakeGlobalPlan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__BUILDER_HPP_
#define PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/srv/detail/make_global_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace srv
{

namespace builder
{

class Init_MakeGlobalPlan_Request_tolerance
{
public:
  explicit Init_MakeGlobalPlan_Request_tolerance(::plan_msgs::srv::MakeGlobalPlan_Request & msg)
  : msg_(msg)
  {}
  ::plan_msgs::srv::MakeGlobalPlan_Request tolerance(::plan_msgs::srv::MakeGlobalPlan_Request::_tolerance_type arg)
  {
    msg_.tolerance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::srv::MakeGlobalPlan_Request msg_;
};

class Init_MakeGlobalPlan_Request_goal
{
public:
  explicit Init_MakeGlobalPlan_Request_goal(::plan_msgs::srv::MakeGlobalPlan_Request & msg)
  : msg_(msg)
  {}
  Init_MakeGlobalPlan_Request_tolerance goal(::plan_msgs::srv::MakeGlobalPlan_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_MakeGlobalPlan_Request_tolerance(msg_);
  }

private:
  ::plan_msgs::srv::MakeGlobalPlan_Request msg_;
};

class Init_MakeGlobalPlan_Request_start
{
public:
  Init_MakeGlobalPlan_Request_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MakeGlobalPlan_Request_goal start(::plan_msgs::srv::MakeGlobalPlan_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_MakeGlobalPlan_Request_goal(msg_);
  }

private:
  ::plan_msgs::srv::MakeGlobalPlan_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::srv::MakeGlobalPlan_Request>()
{
  return plan_msgs::srv::builder::Init_MakeGlobalPlan_Request_start();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace srv
{

namespace builder
{

class Init_MakeGlobalPlan_Response_plan
{
public:
  Init_MakeGlobalPlan_Response_plan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::srv::MakeGlobalPlan_Response plan(::plan_msgs::srv::MakeGlobalPlan_Response::_plan_type arg)
  {
    msg_.plan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::srv::MakeGlobalPlan_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::srv::MakeGlobalPlan_Response>()
{
  return plan_msgs::srv::builder::Init_MakeGlobalPlan_Response_plan();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__BUILDER_HPP_
