// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:srv/MakeGlobalPlan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__STRUCT_H_
#define PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start'
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/MakeGlobalPlan in the package plan_msgs.
typedef struct plan_msgs__srv__MakeGlobalPlan_Request
{
  geometry_msgs__msg__PoseStamped start;
  geometry_msgs__msg__PoseStamped goal;
  float tolerance;
} plan_msgs__srv__MakeGlobalPlan_Request;

// Struct for a sequence of plan_msgs__srv__MakeGlobalPlan_Request.
typedef struct plan_msgs__srv__MakeGlobalPlan_Request__Sequence
{
  plan_msgs__srv__MakeGlobalPlan_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__srv__MakeGlobalPlan_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'plan'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in srv/MakeGlobalPlan in the package plan_msgs.
typedef struct plan_msgs__srv__MakeGlobalPlan_Response
{
  nav_msgs__msg__Path plan;
} plan_msgs__srv__MakeGlobalPlan_Response;

// Struct for a sequence of plan_msgs__srv__MakeGlobalPlan_Response.
typedef struct plan_msgs__srv__MakeGlobalPlan_Response__Sequence
{
  plan_msgs__srv__MakeGlobalPlan_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__srv__MakeGlobalPlan_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__STRUCT_H_
