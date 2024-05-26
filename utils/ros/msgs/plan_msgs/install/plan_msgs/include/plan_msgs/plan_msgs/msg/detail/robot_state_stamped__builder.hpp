// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/RobotStateStamped.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_STATE_STAMPED__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/robot_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotStateStamped_state
{
public:
  explicit Init_RobotStateStamped_state(::plan_msgs::msg::RobotStateStamped & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::RobotStateStamped state(::plan_msgs::msg::RobotStateStamped::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::RobotStateStamped msg_;
};

class Init_RobotStateStamped_header
{
public:
  Init_RobotStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStateStamped_state header(::plan_msgs::msg::RobotStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotStateStamped_state(msg_);
  }

private:
  ::plan_msgs::msg::RobotStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::RobotStateStamped>()
{
  return plan_msgs::msg::builder::Init_RobotStateStamped_header();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_STATE_STAMPED__BUILDER_HPP_
