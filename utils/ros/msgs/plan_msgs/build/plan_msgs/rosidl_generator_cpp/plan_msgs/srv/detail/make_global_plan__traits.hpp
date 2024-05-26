// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:srv/MakeGlobalPlan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__TRAITS_HPP_
#define PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/srv/detail/make_global_plan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start'
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace plan_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MakeGlobalPlan_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start
  {
    out << "start: ";
    to_flow_style_yaml(msg.start, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
    out << ", ";
  }

  // member: tolerance
  {
    out << "tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MakeGlobalPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start:\n";
    to_block_style_yaml(msg.start, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }

  // member: tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MakeGlobalPlan_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::srv::MakeGlobalPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::srv::MakeGlobalPlan_Request & msg)
{
  return plan_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::srv::MakeGlobalPlan_Request>()
{
  return "plan_msgs::srv::MakeGlobalPlan_Request";
}

template<>
inline const char * name<plan_msgs::srv::MakeGlobalPlan_Request>()
{
  return "plan_msgs/srv/MakeGlobalPlan_Request";
}

template<>
struct has_fixed_size<plan_msgs::srv::MakeGlobalPlan_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<plan_msgs::srv::MakeGlobalPlan_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<plan_msgs::srv::MakeGlobalPlan_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'plan'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace plan_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MakeGlobalPlan_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: plan
  {
    out << "plan: ";
    to_flow_style_yaml(msg.plan, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MakeGlobalPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: plan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plan:\n";
    to_block_style_yaml(msg.plan, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MakeGlobalPlan_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::srv::MakeGlobalPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::srv::MakeGlobalPlan_Response & msg)
{
  return plan_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::srv::MakeGlobalPlan_Response>()
{
  return "plan_msgs::srv::MakeGlobalPlan_Response";
}

template<>
inline const char * name<plan_msgs::srv::MakeGlobalPlan_Response>()
{
  return "plan_msgs/srv/MakeGlobalPlan_Response";
}

template<>
struct has_fixed_size<plan_msgs::srv::MakeGlobalPlan_Response>
  : std::integral_constant<bool, has_fixed_size<nav_msgs::msg::Path>::value> {};

template<>
struct has_bounded_size<plan_msgs::srv::MakeGlobalPlan_Response>
  : std::integral_constant<bool, has_bounded_size<nav_msgs::msg::Path>::value> {};

template<>
struct is_message<plan_msgs::srv::MakeGlobalPlan_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<plan_msgs::srv::MakeGlobalPlan>()
{
  return "plan_msgs::srv::MakeGlobalPlan";
}

template<>
inline const char * name<plan_msgs::srv::MakeGlobalPlan>()
{
  return "plan_msgs/srv/MakeGlobalPlan";
}

template<>
struct has_fixed_size<plan_msgs::srv::MakeGlobalPlan>
  : std::integral_constant<
    bool,
    has_fixed_size<plan_msgs::srv::MakeGlobalPlan_Request>::value &&
    has_fixed_size<plan_msgs::srv::MakeGlobalPlan_Response>::value
  >
{
};

template<>
struct has_bounded_size<plan_msgs::srv::MakeGlobalPlan>
  : std::integral_constant<
    bool,
    has_bounded_size<plan_msgs::srv::MakeGlobalPlan_Request>::value &&
    has_bounded_size<plan_msgs::srv::MakeGlobalPlan_Response>::value
  >
{
};

template<>
struct is_service<plan_msgs::srv::MakeGlobalPlan>
  : std::true_type
{
};

template<>
struct is_service_request<plan_msgs::srv::MakeGlobalPlan_Request>
  : std::true_type
{
};

template<>
struct is_service_response<plan_msgs::srv::MakeGlobalPlan_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__TRAITS_HPP_
