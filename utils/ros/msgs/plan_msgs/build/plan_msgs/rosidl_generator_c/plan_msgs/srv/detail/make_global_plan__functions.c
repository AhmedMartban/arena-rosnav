// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_msgs:srv/MakeGlobalPlan.idl
// generated code does not contain a copyright notice
#include "plan_msgs/srv/detail/make_global_plan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `start`
// Member `goal`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
plan_msgs__srv__MakeGlobalPlan_Request__init(plan_msgs__srv__MakeGlobalPlan_Request * msg)
{
  if (!msg) {
    return false;
  }
  // start
  if (!geometry_msgs__msg__PoseStamped__init(&msg->start)) {
    plan_msgs__srv__MakeGlobalPlan_Request__fini(msg);
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PoseStamped__init(&msg->goal)) {
    plan_msgs__srv__MakeGlobalPlan_Request__fini(msg);
    return false;
  }
  // tolerance
  return true;
}

void
plan_msgs__srv__MakeGlobalPlan_Request__fini(plan_msgs__srv__MakeGlobalPlan_Request * msg)
{
  if (!msg) {
    return;
  }
  // start
  geometry_msgs__msg__PoseStamped__fini(&msg->start);
  // goal
  geometry_msgs__msg__PoseStamped__fini(&msg->goal);
  // tolerance
}

bool
plan_msgs__srv__MakeGlobalPlan_Request__are_equal(const plan_msgs__srv__MakeGlobalPlan_Request * lhs, const plan_msgs__srv__MakeGlobalPlan_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->start), &(rhs->start)))
  {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  // tolerance
  if (lhs->tolerance != rhs->tolerance) {
    return false;
  }
  return true;
}

bool
plan_msgs__srv__MakeGlobalPlan_Request__copy(
  const plan_msgs__srv__MakeGlobalPlan_Request * input,
  plan_msgs__srv__MakeGlobalPlan_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // start
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->start), &(output->start)))
  {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  // tolerance
  output->tolerance = input->tolerance;
  return true;
}

plan_msgs__srv__MakeGlobalPlan_Request *
plan_msgs__srv__MakeGlobalPlan_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__srv__MakeGlobalPlan_Request * msg = (plan_msgs__srv__MakeGlobalPlan_Request *)allocator.allocate(sizeof(plan_msgs__srv__MakeGlobalPlan_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__srv__MakeGlobalPlan_Request));
  bool success = plan_msgs__srv__MakeGlobalPlan_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__srv__MakeGlobalPlan_Request__destroy(plan_msgs__srv__MakeGlobalPlan_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__srv__MakeGlobalPlan_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__init(plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__srv__MakeGlobalPlan_Request * data = NULL;

  if (size) {
    data = (plan_msgs__srv__MakeGlobalPlan_Request *)allocator.zero_allocate(size, sizeof(plan_msgs__srv__MakeGlobalPlan_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__srv__MakeGlobalPlan_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__srv__MakeGlobalPlan_Request__fini(&data[i - 1]);
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
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__fini(plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array)
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
      plan_msgs__srv__MakeGlobalPlan_Request__fini(&array->data[i]);
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

plan_msgs__srv__MakeGlobalPlan_Request__Sequence *
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array = (plan_msgs__srv__MakeGlobalPlan_Request__Sequence *)allocator.allocate(sizeof(plan_msgs__srv__MakeGlobalPlan_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__srv__MakeGlobalPlan_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__destroy(plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__srv__MakeGlobalPlan_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__are_equal(const plan_msgs__srv__MakeGlobalPlan_Request__Sequence * lhs, const plan_msgs__srv__MakeGlobalPlan_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__srv__MakeGlobalPlan_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__copy(
  const plan_msgs__srv__MakeGlobalPlan_Request__Sequence * input,
  plan_msgs__srv__MakeGlobalPlan_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__srv__MakeGlobalPlan_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__srv__MakeGlobalPlan_Request * data =
      (plan_msgs__srv__MakeGlobalPlan_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__srv__MakeGlobalPlan_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__srv__MakeGlobalPlan_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__srv__MakeGlobalPlan_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `plan`
#include "nav_msgs/msg/detail/path__functions.h"

bool
plan_msgs__srv__MakeGlobalPlan_Response__init(plan_msgs__srv__MakeGlobalPlan_Response * msg)
{
  if (!msg) {
    return false;
  }
  // plan
  if (!nav_msgs__msg__Path__init(&msg->plan)) {
    plan_msgs__srv__MakeGlobalPlan_Response__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__srv__MakeGlobalPlan_Response__fini(plan_msgs__srv__MakeGlobalPlan_Response * msg)
{
  if (!msg) {
    return;
  }
  // plan
  nav_msgs__msg__Path__fini(&msg->plan);
}

bool
plan_msgs__srv__MakeGlobalPlan_Response__are_equal(const plan_msgs__srv__MakeGlobalPlan_Response * lhs, const plan_msgs__srv__MakeGlobalPlan_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // plan
  if (!nav_msgs__msg__Path__are_equal(
      &(lhs->plan), &(rhs->plan)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__srv__MakeGlobalPlan_Response__copy(
  const plan_msgs__srv__MakeGlobalPlan_Response * input,
  plan_msgs__srv__MakeGlobalPlan_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // plan
  if (!nav_msgs__msg__Path__copy(
      &(input->plan), &(output->plan)))
  {
    return false;
  }
  return true;
}

plan_msgs__srv__MakeGlobalPlan_Response *
plan_msgs__srv__MakeGlobalPlan_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__srv__MakeGlobalPlan_Response * msg = (plan_msgs__srv__MakeGlobalPlan_Response *)allocator.allocate(sizeof(plan_msgs__srv__MakeGlobalPlan_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__srv__MakeGlobalPlan_Response));
  bool success = plan_msgs__srv__MakeGlobalPlan_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__srv__MakeGlobalPlan_Response__destroy(plan_msgs__srv__MakeGlobalPlan_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__srv__MakeGlobalPlan_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__init(plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__srv__MakeGlobalPlan_Response * data = NULL;

  if (size) {
    data = (plan_msgs__srv__MakeGlobalPlan_Response *)allocator.zero_allocate(size, sizeof(plan_msgs__srv__MakeGlobalPlan_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__srv__MakeGlobalPlan_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__srv__MakeGlobalPlan_Response__fini(&data[i - 1]);
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
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__fini(plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array)
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
      plan_msgs__srv__MakeGlobalPlan_Response__fini(&array->data[i]);
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

plan_msgs__srv__MakeGlobalPlan_Response__Sequence *
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array = (plan_msgs__srv__MakeGlobalPlan_Response__Sequence *)allocator.allocate(sizeof(plan_msgs__srv__MakeGlobalPlan_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__srv__MakeGlobalPlan_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__destroy(plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__srv__MakeGlobalPlan_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__are_equal(const plan_msgs__srv__MakeGlobalPlan_Response__Sequence * lhs, const plan_msgs__srv__MakeGlobalPlan_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__srv__MakeGlobalPlan_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__copy(
  const plan_msgs__srv__MakeGlobalPlan_Response__Sequence * input,
  plan_msgs__srv__MakeGlobalPlan_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__srv__MakeGlobalPlan_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__srv__MakeGlobalPlan_Response * data =
      (plan_msgs__srv__MakeGlobalPlan_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__srv__MakeGlobalPlan_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__srv__MakeGlobalPlan_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__srv__MakeGlobalPlan_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
