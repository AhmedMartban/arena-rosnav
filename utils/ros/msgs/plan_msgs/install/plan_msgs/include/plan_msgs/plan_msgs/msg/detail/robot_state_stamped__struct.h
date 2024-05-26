// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/RobotStateStamped.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_STATE_STAMPED__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_STATE_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "plan_msgs/msg/detail/robot_state__struct.h"

/// Struct defined in msg/RobotStateStamped in the package plan_msgs.
typedef struct plan_msgs__msg__RobotStateStamped
{
  std_msgs__msg__Header header;
  plan_msgs__msg__RobotState state;
} plan_msgs__msg__RobotStateStamped;

// Struct for a sequence of plan_msgs__msg__RobotStateStamped.
typedef struct plan_msgs__msg__RobotStateStamped__Sequence
{
  plan_msgs__msg__RobotStateStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__RobotStateStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_STATE_STAMPED__STRUCT_H_
