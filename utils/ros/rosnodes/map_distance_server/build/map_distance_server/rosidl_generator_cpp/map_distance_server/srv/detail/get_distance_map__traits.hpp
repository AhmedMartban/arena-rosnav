// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from map_distance_server:srv/GetDistanceMap.idl
// generated code does not contain a copyright notice

#ifndef MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__TRAITS_HPP_
#define MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "map_distance_server/srv/detail/get_distance_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace map_distance_server
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetDistanceMap_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetDistanceMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetDistanceMap_Request & msg, bool use_flow_style = false)
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

}  // namespace map_distance_server

namespace rosidl_generator_traits
{

[[deprecated("use map_distance_server::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const map_distance_server::srv::GetDistanceMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  map_distance_server::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use map_distance_server::srv::to_yaml() instead")]]
inline std::string to_yaml(const map_distance_server::srv::GetDistanceMap_Request & msg)
{
  return map_distance_server::srv::to_yaml(msg);
}

template<>
inline const char * data_type<map_distance_server::srv::GetDistanceMap_Request>()
{
  return "map_distance_server::srv::GetDistanceMap_Request";
}

template<>
inline const char * name<map_distance_server::srv::GetDistanceMap_Request>()
{
  return "map_distance_server/srv/GetDistanceMap_Request";
}

template<>
struct has_fixed_size<map_distance_server::srv::GetDistanceMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<map_distance_server::srv::GetDistanceMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<map_distance_server::srv::GetDistanceMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'info'
#include "nav_msgs/msg/detail/map_meta_data__traits.hpp"

namespace map_distance_server
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetDistanceMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetDistanceMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetDistanceMap_Response & msg, bool use_flow_style = false)
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

}  // namespace map_distance_server

namespace rosidl_generator_traits
{

[[deprecated("use map_distance_server::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const map_distance_server::srv::GetDistanceMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  map_distance_server::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use map_distance_server::srv::to_yaml() instead")]]
inline std::string to_yaml(const map_distance_server::srv::GetDistanceMap_Response & msg)
{
  return map_distance_server::srv::to_yaml(msg);
}

template<>
inline const char * data_type<map_distance_server::srv::GetDistanceMap_Response>()
{
  return "map_distance_server::srv::GetDistanceMap_Response";
}

template<>
inline const char * name<map_distance_server::srv::GetDistanceMap_Response>()
{
  return "map_distance_server/srv/GetDistanceMap_Response";
}

template<>
struct has_fixed_size<map_distance_server::srv::GetDistanceMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<map_distance_server::srv::GetDistanceMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<map_distance_server::srv::GetDistanceMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<map_distance_server::srv::GetDistanceMap>()
{
  return "map_distance_server::srv::GetDistanceMap";
}

template<>
inline const char * name<map_distance_server::srv::GetDistanceMap>()
{
  return "map_distance_server/srv/GetDistanceMap";
}

template<>
struct has_fixed_size<map_distance_server::srv::GetDistanceMap>
  : std::integral_constant<
    bool,
    has_fixed_size<map_distance_server::srv::GetDistanceMap_Request>::value &&
    has_fixed_size<map_distance_server::srv::GetDistanceMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<map_distance_server::srv::GetDistanceMap>
  : std::integral_constant<
    bool,
    has_bounded_size<map_distance_server::srv::GetDistanceMap_Request>::value &&
    has_bounded_size<map_distance_server::srv::GetDistanceMap_Response>::value
  >
{
};

template<>
struct is_service<map_distance_server::srv::GetDistanceMap>
  : std::true_type
{
};

template<>
struct is_service_request<map_distance_server::srv::GetDistanceMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<map_distance_server::srv::GetDistanceMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__TRAITS_HPP_
