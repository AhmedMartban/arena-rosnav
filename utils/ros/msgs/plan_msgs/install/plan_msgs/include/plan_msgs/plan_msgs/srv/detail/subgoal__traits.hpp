// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:srv/Subgoal.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__SUBGOAL__TRAITS_HPP_
#define PLAN_MSGS__SRV__DETAIL__SUBGOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/srv/detail/subgoal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace plan_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Subgoal_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Subgoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Subgoal_Request & msg, bool use_flow_style = false)
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
  const plan_msgs::srv::Subgoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::srv::Subgoal_Request & msg)
{
  return plan_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::srv::Subgoal_Request>()
{
  return "plan_msgs::srv::Subgoal_Request";
}

template<>
inline const char * name<plan_msgs::srv::Subgoal_Request>()
{
  return "plan_msgs/srv/Subgoal_Request";
}

template<>
struct has_fixed_size<plan_msgs::srv::Subgoal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<plan_msgs::srv::Subgoal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<plan_msgs::srv::Subgoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'subgoal'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace plan_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Subgoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: subgoal
  {
    out << "subgoal: ";
    to_flow_style_yaml(msg.subgoal, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Subgoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: subgoal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subgoal:\n";
    to_block_style_yaml(msg.subgoal, out, indentation + 2);
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Subgoal_Response & msg, bool use_flow_style = false)
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
  const plan_msgs::srv::Subgoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::srv::Subgoal_Response & msg)
{
  return plan_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::srv::Subgoal_Response>()
{
  return "plan_msgs::srv::Subgoal_Response";
}

template<>
inline const char * name<plan_msgs::srv::Subgoal_Response>()
{
  return "plan_msgs/srv/Subgoal_Response";
}

template<>
struct has_fixed_size<plan_msgs::srv::Subgoal_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::srv::Subgoal_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::srv::Subgoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<plan_msgs::srv::Subgoal>()
{
  return "plan_msgs::srv::Subgoal";
}

template<>
inline const char * name<plan_msgs::srv::Subgoal>()
{
  return "plan_msgs/srv/Subgoal";
}

template<>
struct has_fixed_size<plan_msgs::srv::Subgoal>
  : std::integral_constant<
    bool,
    has_fixed_size<plan_msgs::srv::Subgoal_Request>::value &&
    has_fixed_size<plan_msgs::srv::Subgoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<plan_msgs::srv::Subgoal>
  : std::integral_constant<
    bool,
    has_bounded_size<plan_msgs::srv::Subgoal_Request>::value &&
    has_bounded_size<plan_msgs::srv::Subgoal_Response>::value
  >
{
};

template<>
struct is_service<plan_msgs::srv::Subgoal>
  : std::true_type
{
};

template<>
struct is_service_request<plan_msgs::srv::Subgoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<plan_msgs::srv::Subgoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__SRV__DETAIL__SUBGOAL__TRAITS_HPP_
