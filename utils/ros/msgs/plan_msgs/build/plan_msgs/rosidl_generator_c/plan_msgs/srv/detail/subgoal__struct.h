// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:srv/Subgoal.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__SUBGOAL__STRUCT_H_
#define PLAN_MSGS__SRV__DETAIL__SUBGOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Subgoal in the package plan_msgs.
typedef struct plan_msgs__srv__Subgoal_Request
{
  uint8_t structure_needs_at_least_one_member;
} plan_msgs__srv__Subgoal_Request;

// Struct for a sequence of plan_msgs__srv__Subgoal_Request.
typedef struct plan_msgs__srv__Subgoal_Request__Sequence
{
  plan_msgs__srv__Subgoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__srv__Subgoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'subgoal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Subgoal in the package plan_msgs.
typedef struct plan_msgs__srv__Subgoal_Response
{
  geometry_msgs__msg__PoseStamped subgoal;
  bool success;
  rosidl_runtime_c__String message;
} plan_msgs__srv__Subgoal_Response;

// Struct for a sequence of plan_msgs__srv__Subgoal_Response.
typedef struct plan_msgs__srv__Subgoal_Response__Sequence
{
  plan_msgs__srv__Subgoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__srv__Subgoal_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__SRV__DETAIL__SUBGOAL__STRUCT_H_
