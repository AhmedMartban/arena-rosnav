// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from map_distance_server:srv/GetDistanceMap.idl
// generated code does not contain a copyright notice
#include "map_distance_server/srv/detail/get_distance_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
map_distance_server__srv__GetDistanceMap_Request__init(map_distance_server__srv__GetDistanceMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
map_distance_server__srv__GetDistanceMap_Request__fini(map_distance_server__srv__GetDistanceMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
map_distance_server__srv__GetDistanceMap_Request__are_equal(const map_distance_server__srv__GetDistanceMap_Request * lhs, const map_distance_server__srv__GetDistanceMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
map_distance_server__srv__GetDistanceMap_Request__copy(
  const map_distance_server__srv__GetDistanceMap_Request * input,
  map_distance_server__srv__GetDistanceMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

map_distance_server__srv__GetDistanceMap_Request *
map_distance_server__srv__GetDistanceMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  map_distance_server__srv__GetDistanceMap_Request * msg = (map_distance_server__srv__GetDistanceMap_Request *)allocator.allocate(sizeof(map_distance_server__srv__GetDistanceMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(map_distance_server__srv__GetDistanceMap_Request));
  bool success = map_distance_server__srv__GetDistanceMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
map_distance_server__srv__GetDistanceMap_Request__destroy(map_distance_server__srv__GetDistanceMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    map_distance_server__srv__GetDistanceMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
map_distance_server__srv__GetDistanceMap_Request__Sequence__init(map_distance_server__srv__GetDistanceMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  map_distance_server__srv__GetDistanceMap_Request * data = NULL;

  if (size) {
    data = (map_distance_server__srv__GetDistanceMap_Request *)allocator.zero_allocate(size, sizeof(map_distance_server__srv__GetDistanceMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = map_distance_server__srv__GetDistanceMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        map_distance_server__srv__GetDistanceMap_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
map_distance_server__srv__GetDistanceMap_Request__Sequence__fini(map_distance_server__srv__GetDistanceMap_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      map_distance_server__srv__GetDistanceMap_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

map_distance_server__srv__GetDistanceMap_Request__Sequence *
map_distance_server__srv__GetDistanceMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  map_distance_server__srv__GetDistanceMap_Request__Sequence * array = (map_distance_server__srv__GetDistanceMap_Request__Sequence *)allocator.allocate(sizeof(map_distance_server__srv__GetDistanceMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = map_distance_server__srv__GetDistanceMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
map_distance_server__srv__GetDistanceMap_Request__Sequence__destroy(map_distance_server__srv__GetDistanceMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    map_distance_server__srv__GetDistanceMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
map_distance_server__srv__GetDistanceMap_Request__Sequence__are_equal(const map_distance_server__srv__GetDistanceMap_Request__Sequence * lhs, const map_distance_server__srv__GetDistanceMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!map_distance_server__srv__GetDistanceMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
map_distance_server__srv__GetDistanceMap_Request__Sequence__copy(
  const map_distance_server__srv__GetDistanceMap_Request__Sequence * input,
  map_distance_server__srv__GetDistanceMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(map_distance_server__srv__GetDistanceMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    map_distance_server__srv__GetDistanceMap_Request * data =
      (map_distance_server__srv__GetDistanceMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!map_distance_server__srv__GetDistanceMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          map_distance_server__srv__GetDistanceMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!map_distance_server__srv__GetDistanceMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `info`
#include "nav_msgs/msg/detail/map_meta_data__functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
map_distance_server__srv__GetDistanceMap_Response__init(map_distance_server__srv__GetDistanceMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    map_distance_server__srv__GetDistanceMap_Response__fini(msg);
    return false;
  }
  // info
  if (!nav_msgs__msg__MapMetaData__init(&msg->info)) {
    map_distance_server__srv__GetDistanceMap_Response__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->data, 0)) {
    map_distance_server__srv__GetDistanceMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
map_distance_server__srv__GetDistanceMap_Response__fini(map_distance_server__srv__GetDistanceMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // info
  nav_msgs__msg__MapMetaData__fini(&msg->info);
  // data
  rosidl_runtime_c__int32__Sequence__fini(&msg->data);
}

bool
map_distance_server__srv__GetDistanceMap_Response__are_equal(const map_distance_server__srv__GetDistanceMap_Response * lhs, const map_distance_server__srv__GetDistanceMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // info
  if (!nav_msgs__msg__MapMetaData__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
map_distance_server__srv__GetDistanceMap_Response__copy(
  const map_distance_server__srv__GetDistanceMap_Response * input,
  map_distance_server__srv__GetDistanceMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // info
  if (!nav_msgs__msg__MapMetaData__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

map_distance_server__srv__GetDistanceMap_Response *
map_distance_server__srv__GetDistanceMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  map_distance_server__srv__GetDistanceMap_Response * msg = (map_distance_server__srv__GetDistanceMap_Response *)allocator.allocate(sizeof(map_distance_server__srv__GetDistanceMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(map_distance_server__srv__GetDistanceMap_Response));
  bool success = map_distance_server__srv__GetDistanceMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
map_distance_server__srv__GetDistanceMap_Response__destroy(map_distance_server__srv__GetDistanceMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    map_distance_server__srv__GetDistanceMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
map_distance_server__srv__GetDistanceMap_Response__Sequence__init(map_distance_server__srv__GetDistanceMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  map_distance_server__srv__GetDistanceMap_Response * data = NULL;

  if (size) {
    data = (map_distance_server__srv__GetDistanceMap_Response *)allocator.zero_allocate(size, sizeof(map_distance_server__srv__GetDistanceMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = map_distance_server__srv__GetDistanceMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        map_distance_server__srv__GetDistanceMap_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
map_distance_server__srv__GetDistanceMap_Response__Sequence__fini(map_distance_server__srv__GetDistanceMap_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      map_distance_server__srv__GetDistanceMap_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

map_distance_server__srv__GetDistanceMap_Response__Sequence *
map_distance_server__srv__GetDistanceMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  map_distance_server__srv__GetDistanceMap_Response__Sequence * array = (map_distance_server__srv__GetDistanceMap_Response__Sequence *)allocator.allocate(sizeof(map_distance_server__srv__GetDistanceMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = map_distance_server__srv__GetDistanceMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
map_distance_server__srv__GetDistanceMap_Response__Sequence__destroy(map_distance_server__srv__GetDistanceMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    map_distance_server__srv__GetDistanceMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
map_distance_server__srv__GetDistanceMap_Response__Sequence__are_equal(const map_distance_server__srv__GetDistanceMap_Response__Sequence * lhs, const map_distance_server__srv__GetDistanceMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!map_distance_server__srv__GetDistanceMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
map_distance_server__srv__GetDistanceMap_Response__Sequence__copy(
  const map_distance_server__srv__GetDistanceMap_Response__Sequence * input,
  map_distance_server__srv__GetDistanceMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(map_distance_server__srv__GetDistanceMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    map_distance_server__srv__GetDistanceMap_Response * data =
      (map_distance_server__srv__GetDistanceMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!map_distance_server__srv__GetDistanceMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          map_distance_server__srv__GetDistanceMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!map_distance_server__srv__GetDistanceMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
