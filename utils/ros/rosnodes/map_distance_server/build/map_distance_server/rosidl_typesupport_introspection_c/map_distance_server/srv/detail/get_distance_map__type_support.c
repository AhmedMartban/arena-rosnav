// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from map_distance_server:srv/GetDistanceMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "map_distance_server/srv/detail/get_distance_map__rosidl_typesupport_introspection_c.h"
#include "map_distance_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "map_distance_server/srv/detail/get_distance_map__functions.h"
#include "map_distance_server/srv/detail/get_distance_map__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  map_distance_server__srv__GetDistanceMap_Request__init(message_memory);
}

void map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_fini_function(void * message_memory)
{
  map_distance_server__srv__GetDistanceMap_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(map_distance_server__srv__GetDistanceMap_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_members = {
  "map_distance_server__srv",  // message namespace
  "GetDistanceMap_Request",  // message name
  1,  // number of fields
  sizeof(map_distance_server__srv__GetDistanceMap_Request),
  map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_member_array,  // message members
  map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_type_support_handle = {
  0,
  &map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_map_distance_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap_Request)() {
  if (!map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_type_support_handle.typesupport_identifier) {
    map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &map_distance_server__srv__GetDistanceMap_Request__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "map_distance_server/srv/detail/get_distance_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "map_distance_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "map_distance_server/srv/detail/get_distance_map__functions.h"
// already included above
// #include "map_distance_server/srv/detail/get_distance_map__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `info`
#include "nav_msgs/msg/map_meta_data.h"
// Member `info`
#include "nav_msgs/msg/detail/map_meta_data__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  map_distance_server__srv__GetDistanceMap_Response__init(message_memory);
}

void map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_fini_function(void * message_memory)
{
  map_distance_server__srv__GetDistanceMap_Response__fini(message_memory);
}

size_t map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__size_function__GetDistanceMap_Response__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetDistanceMap_Response__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__get_function__GetDistanceMap_Response__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__fetch_function__GetDistanceMap_Response__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetDistanceMap_Response__data(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__assign_function__GetDistanceMap_Response__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__get_function__GetDistanceMap_Response__data(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__resize_function__GetDistanceMap_Response__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(map_distance_server__srv__GetDistanceMap_Response, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(map_distance_server__srv__GetDistanceMap_Response, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(map_distance_server__srv__GetDistanceMap_Response, data),  // bytes offset in struct
    NULL,  // default value
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__size_function__GetDistanceMap_Response__data,  // size() function pointer
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetDistanceMap_Response__data,  // get_const(index) function pointer
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__get_function__GetDistanceMap_Response__data,  // get(index) function pointer
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__fetch_function__GetDistanceMap_Response__data,  // fetch(index, &value) function pointer
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__assign_function__GetDistanceMap_Response__data,  // assign(index, value) function pointer
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__resize_function__GetDistanceMap_Response__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_members = {
  "map_distance_server__srv",  // message namespace
  "GetDistanceMap_Response",  // message name
  3,  // number of fields
  sizeof(map_distance_server__srv__GetDistanceMap_Response),
  map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_member_array,  // message members
  map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_type_support_handle = {
  0,
  &map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_map_distance_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap_Response)() {
  map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, MapMetaData)();
  if (!map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_type_support_handle.typesupport_identifier) {
    map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &map_distance_server__srv__GetDistanceMap_Response__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "map_distance_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "map_distance_server/srv/detail/get_distance_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_members = {
  "map_distance_server__srv",  // service namespace
  "GetDistanceMap",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_Request_message_type_support_handle,
  NULL  // response message
  // map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_Response_message_type_support_handle
};

static rosidl_service_type_support_t map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_type_support_handle = {
  0,
  &map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_map_distance_server
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap)() {
  if (!map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_type_support_handle.typesupport_identifier) {
    map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, map_distance_server, srv, GetDistanceMap_Response)()->data;
  }

  return &map_distance_server__srv__detail__get_distance_map__rosidl_typesupport_introspection_c__GetDistanceMap_service_type_support_handle;
}
