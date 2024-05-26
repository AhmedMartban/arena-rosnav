// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:srv/Subgoal.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__SUBGOAL__BUILDER_HPP_
#define PLAN_MSGS__SRV__DETAIL__SUBGOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/srv/detail/subgoal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::srv::Subgoal_Request>()
{
  return ::plan_msgs::srv::Subgoal_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace srv
{

namespace builder
{

class Init_Subgoal_Response_message
{
public:
  explicit Init_Subgoal_Response_message(::plan_msgs::srv::Subgoal_Response & msg)
  : msg_(msg)
  {}
  ::plan_msgs::srv::Subgoal_Response message(::plan_msgs::srv::Subgoal_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::srv::Subgoal_Response msg_;
};

class Init_Subgoal_Response_success
{
public:
  explicit Init_Subgoal_Response_success(::plan_msgs::srv::Subgoal_Response & msg)
  : msg_(msg)
  {}
  Init_Subgoal_Response_message success(::plan_msgs::srv::Subgoal_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Subgoal_Response_message(msg_);
  }

private:
  ::plan_msgs::srv::Subgoal_Response msg_;
};

class Init_Subgoal_Response_subgoal
{
public:
  Init_Subgoal_Response_subgoal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Subgoal_Response_success subgoal(::plan_msgs::srv::Subgoal_Response::_subgoal_type arg)
  {
    msg_.subgoal = std::move(arg);
    return Init_Subgoal_Response_success(msg_);
  }

private:
  ::plan_msgs::srv::Subgoal_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::srv::Subgoal_Response>()
{
  return plan_msgs::srv::builder::Init_Subgoal_Response_subgoal();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__SRV__DETAIL__SUBGOAL__BUILDER_HPP_
