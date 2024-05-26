// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_twist
{
public:
  explicit Init_RobotState_twist(::plan_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::RobotState twist(::plan_msgs::msg::RobotState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::RobotState msg_;
};

class Init_RobotState_pose
{
public:
  Init_RobotState_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_twist pose(::plan_msgs::msg::RobotState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_RobotState_twist(msg_);
  }

private:
  ::plan_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::RobotState>()
{
  return plan_msgs::msg::builder::Init_RobotState_pose();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
