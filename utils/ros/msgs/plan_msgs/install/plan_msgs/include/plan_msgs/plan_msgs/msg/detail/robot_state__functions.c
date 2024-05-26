// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "plan_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
plan_msgs__msg__RobotState__init(plan_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    plan_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    plan_msgs__msg__RobotState__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__msg__RobotState__fini(plan_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
}

bool
plan_msgs__msg__RobotState__are_equal(const plan_msgs__msg__RobotState * lhs, const plan_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__msg__RobotState__copy(
  const plan_msgs__msg__RobotState * input,
  plan_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  return true;
}

plan_msgs__msg__RobotState *
plan_msgs__msg__RobotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__RobotState * msg = (plan_msgs__msg__RobotState *)allocator.allocate(sizeof(plan_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__msg__RobotState));
  bool success = plan_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__msg__RobotState__destroy(plan_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__msg__RobotState__Sequence__init(plan_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__RobotState * data = NULL;

  if (size) {
    data = (plan_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(plan_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__msg__RobotState__fini(&data[i - 1]);
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
plan_msgs__msg__RobotState__Sequence__fini(plan_msgs__msg__RobotState__Sequence * array)
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
      plan_msgs__msg__RobotState__fini(&array->data[i]);
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

plan_msgs__msg__RobotState__Sequence *
plan_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__RobotState__Sequence * array = (plan_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(plan_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__msg__RobotState__Sequence__destroy(plan_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__msg__RobotState__Sequence__are_equal(const plan_msgs__msg__RobotState__Sequence * lhs, const plan_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__msg__RobotState__Sequence__copy(
  const plan_msgs__msg__RobotState__Sequence * input,
  plan_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__msg__RobotState * data =
      (plan_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
