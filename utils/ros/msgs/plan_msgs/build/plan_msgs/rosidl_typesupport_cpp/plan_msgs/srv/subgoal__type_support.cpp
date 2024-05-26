// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from plan_msgs:srv/Subgoal.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "plan_msgs/srv/detail/subgoal__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace plan_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Subgoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Subgoal_Request_type_support_ids_t;

static const _Subgoal_Request_type_support_ids_t _Subgoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Subgoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Subgoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Subgoal_Request_type_support_symbol_names_t _Subgoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, plan_msgs, srv, Subgoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, srv, Subgoal_Request)),
  }
};

typedef struct _Subgoal_Request_type_support_data_t
{
  void * data[2];
} _Subgoal_Request_type_support_data_t;

static _Subgoal_Request_type_support_data_t _Subgoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Subgoal_Request_message_typesupport_map = {
  2,
  "plan_msgs",
  &_Subgoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Subgoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Subgoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Subgoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Subgoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace plan_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<plan_msgs::srv::Subgoal_Request>()
{
  return &::plan_msgs::srv::rosidl_typesupport_cpp::Subgoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, plan_msgs, srv, Subgoal_Request)() {
  return get_message_type_support_handle<plan_msgs::srv::Subgoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "plan_msgs/srv/detail/subgoal__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace plan_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Subgoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Subgoal_Response_type_support_ids_t;

static const _Subgoal_Response_type_support_ids_t _Subgoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Subgoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Subgoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Subgoal_Response_type_support_symbol_names_t _Subgoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, plan_msgs, srv, Subgoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, srv, Subgoal_Response)),
  }
};

typedef struct _Subgoal_Response_type_support_data_t
{
  void * data[2];
} _Subgoal_Response_type_support_data_t;

static _Subgoal_Response_type_support_data_t _Subgoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Subgoal_Response_message_typesupport_map = {
  2,
  "plan_msgs",
  &_Subgoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Subgoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Subgoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Subgoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Subgoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace plan_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<plan_msgs::srv::Subgoal_Response>()
{
  return &::plan_msgs::srv::rosidl_typesupport_cpp::Subgoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, plan_msgs, srv, Subgoal_Response)() {
  return get_message_type_support_handle<plan_msgs::srv::Subgoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "plan_msgs/srv/detail/subgoal__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace plan_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Subgoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Subgoal_type_support_ids_t;

static const _Subgoal_type_support_ids_t _Subgoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Subgoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Subgoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Subgoal_type_support_symbol_names_t _Subgoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, plan_msgs, srv, Subgoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, srv, Subgoal)),
  }
};

typedef struct _Subgoal_type_support_data_t
{
  void * data[2];
} _Subgoal_type_support_data_t;

static _Subgoal_type_support_data_t _Subgoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Subgoal_service_typesupport_map = {
  2,
  "plan_msgs",
  &_Subgoal_service_typesupport_ids.typesupport_identifier[0],
  &_Subgoal_service_typesupport_symbol_names.symbol_name[0],
  &_Subgoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Subgoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Subgoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace plan_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<plan_msgs::srv::Subgoal>()
{
  return &::plan_msgs::srv::rosidl_typesupport_cpp::Subgoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
