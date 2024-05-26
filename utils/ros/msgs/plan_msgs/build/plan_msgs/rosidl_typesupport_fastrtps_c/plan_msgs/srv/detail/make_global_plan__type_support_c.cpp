// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from plan_msgs:srv/MakeGlobalPlan.idl
// generated code does not contain a copyright notice
#include "plan_msgs/srv/detail/make_global_plan__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "plan_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "plan_msgs/srv/detail/make_global_plan__struct.h"
#include "plan_msgs/srv/detail/make_global_plan__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // goal, start

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_msgs
size_t get_serialized_size_geometry_msgs__msg__PoseStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_msgs
size_t max_serialized_size_geometry_msgs__msg__PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped)();


using _MakeGlobalPlan_Request__ros_msg_type = plan_msgs__srv__MakeGlobalPlan_Request;

static bool _MakeGlobalPlan_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MakeGlobalPlan_Request__ros_msg_type * ros_message = static_cast<const _MakeGlobalPlan_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: start
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->start, cdr))
    {
      return false;
    }
  }

  // Field name: goal
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->goal, cdr))
    {
      return false;
    }
  }

  // Field name: tolerance
  {
    cdr << ros_message->tolerance;
  }

  return true;
}

static bool _MakeGlobalPlan_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MakeGlobalPlan_Request__ros_msg_type * ros_message = static_cast<_MakeGlobalPlan_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: start
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->start))
    {
      return false;
    }
  }

  // Field name: goal
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->goal))
    {
      return false;
    }
  }

  // Field name: tolerance
  {
    cdr >> ros_message->tolerance;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_plan_msgs
size_t get_serialized_size_plan_msgs__srv__MakeGlobalPlan_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MakeGlobalPlan_Request__ros_msg_type * ros_message = static_cast<const _MakeGlobalPlan_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name start

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->start), current_alignment);
  // field.name goal

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->goal), current_alignment);
  // field.name tolerance
  {
    size_t item_size = sizeof(ros_message->tolerance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MakeGlobalPlan_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_plan_msgs__srv__MakeGlobalPlan_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_plan_msgs
size_t max_serialized_size_plan_msgs__srv__MakeGlobalPlan_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: start
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: goal
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: tolerance
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = plan_msgs__srv__MakeGlobalPlan_Request;
    is_plain =
      (
      offsetof(DataType, tolerance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MakeGlobalPlan_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_plan_msgs__srv__MakeGlobalPlan_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MakeGlobalPlan_Request = {
  "plan_msgs::srv",
  "MakeGlobalPlan_Request",
  _MakeGlobalPlan_Request__cdr_serialize,
  _MakeGlobalPlan_Request__cdr_deserialize,
  _MakeGlobalPlan_Request__get_serialized_size,
  _MakeGlobalPlan_Request__max_serialized_size
};

static rosidl_message_type_support_t _MakeGlobalPlan_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MakeGlobalPlan_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, plan_msgs, srv, MakeGlobalPlan_Request)() {
  return &_MakeGlobalPlan_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "plan_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "plan_msgs/srv/detail/make_global_plan__struct.h"
// already included above
// #include "plan_msgs/srv/detail/make_global_plan__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nav_msgs/msg/detail/path__functions.h"  // plan

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_msgs
size_t get_serialized_size_nav_msgs__msg__Path(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_msgs
size_t max_serialized_size_nav_msgs__msg__Path(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, nav_msgs, msg, Path)();


using _MakeGlobalPlan_Response__ros_msg_type = plan_msgs__srv__MakeGlobalPlan_Response;

static bool _MakeGlobalPlan_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MakeGlobalPlan_Response__ros_msg_type * ros_message = static_cast<const _MakeGlobalPlan_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: plan
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, nav_msgs, msg, Path
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->plan, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _MakeGlobalPlan_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MakeGlobalPlan_Response__ros_msg_type * ros_message = static_cast<_MakeGlobalPlan_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: plan
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, nav_msgs, msg, Path
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->plan))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_plan_msgs
size_t get_serialized_size_plan_msgs__srv__MakeGlobalPlan_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MakeGlobalPlan_Response__ros_msg_type * ros_message = static_cast<const _MakeGlobalPlan_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name plan

  current_alignment += get_serialized_size_nav_msgs__msg__Path(
    &(ros_message->plan), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _MakeGlobalPlan_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_plan_msgs__srv__MakeGlobalPlan_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_plan_msgs
size_t max_serialized_size_plan_msgs__srv__MakeGlobalPlan_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: plan
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_nav_msgs__msg__Path(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = plan_msgs__srv__MakeGlobalPlan_Response;
    is_plain =
      (
      offsetof(DataType, plan) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MakeGlobalPlan_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_plan_msgs__srv__MakeGlobalPlan_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MakeGlobalPlan_Response = {
  "plan_msgs::srv",
  "MakeGlobalPlan_Response",
  _MakeGlobalPlan_Response__cdr_serialize,
  _MakeGlobalPlan_Response__cdr_deserialize,
  _MakeGlobalPlan_Response__get_serialized_size,
  _MakeGlobalPlan_Response__max_serialized_size
};

static rosidl_message_type_support_t _MakeGlobalPlan_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MakeGlobalPlan_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, plan_msgs, srv, MakeGlobalPlan_Response)() {
  return &_MakeGlobalPlan_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "plan_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "plan_msgs/srv/make_global_plan.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t MakeGlobalPlan__callbacks = {
  "plan_msgs::srv",
  "MakeGlobalPlan",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, plan_msgs, srv, MakeGlobalPlan_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, plan_msgs, srv, MakeGlobalPlan_Response)(),
};

static rosidl_service_type_support_t MakeGlobalPlan__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &MakeGlobalPlan__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, plan_msgs, srv, MakeGlobalPlan)() {
  return &MakeGlobalPlan__handle;
}

#if defined(__cplusplus)
}
#endif
