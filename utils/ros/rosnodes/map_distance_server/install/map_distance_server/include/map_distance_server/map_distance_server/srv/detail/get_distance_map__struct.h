// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from map_distance_server:srv/GetDistanceMap.idl
// generated code does not contain a copyright notice

#ifndef MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__STRUCT_H_
#define MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetDistanceMap in the package map_distance_server.
typedef struct map_distance_server__srv__GetDistanceMap_Request
{
  uint8_t structure_needs_at_least_one_member;
} map_distance_server__srv__GetDistanceMap_Request;

// Struct for a sequence of map_distance_server__srv__GetDistanceMap_Request.
typedef struct map_distance_server__srv__GetDistanceMap_Request__Sequence
{
  map_distance_server__srv__GetDistanceMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} map_distance_server__srv__GetDistanceMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'info'
#include "nav_msgs/msg/detail/map_meta_data__struct.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetDistanceMap in the package map_distance_server.
typedef struct map_distance_server__srv__GetDistanceMap_Response
{
  std_msgs__msg__Header header;
  nav_msgs__msg__MapMetaData info;
  rosidl_runtime_c__int32__Sequence data;
} map_distance_server__srv__GetDistanceMap_Response;

// Struct for a sequence of map_distance_server__srv__GetDistanceMap_Response.
typedef struct map_distance_server__srv__GetDistanceMap_Response__Sequence
{
  map_distance_server__srv__GetDistanceMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} map_distance_server__srv__GetDistanceMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__STRUCT_H_
