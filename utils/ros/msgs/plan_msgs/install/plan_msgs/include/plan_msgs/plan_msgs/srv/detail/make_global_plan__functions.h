// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from plan_msgs:srv/MakeGlobalPlan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__FUNCTIONS_H_
#define PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "plan_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "plan_msgs/srv/detail/make_global_plan__struct.h"

/// Initialize srv/MakeGlobalPlan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * plan_msgs__srv__MakeGlobalPlan_Request
 * )) before or use
 * plan_msgs__srv__MakeGlobalPlan_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Request__init(plan_msgs__srv__MakeGlobalPlan_Request * msg);

/// Finalize srv/MakeGlobalPlan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Request__fini(plan_msgs__srv__MakeGlobalPlan_Request * msg);

/// Create srv/MakeGlobalPlan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * plan_msgs__srv__MakeGlobalPlan_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
plan_msgs__srv__MakeGlobalPlan_Request *
plan_msgs__srv__MakeGlobalPlan_Request__create();

/// Destroy srv/MakeGlobalPlan message.
/**
 * It calls
 * plan_msgs__srv__MakeGlobalPlan_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Request__destroy(plan_msgs__srv__MakeGlobalPlan_Request * msg);

/// Check for srv/MakeGlobalPlan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Request__are_equal(const plan_msgs__srv__MakeGlobalPlan_Request * lhs, const plan_msgs__srv__MakeGlobalPlan_Request * rhs);

/// Copy a srv/MakeGlobalPlan message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Request__copy(
  const plan_msgs__srv__MakeGlobalPlan_Request * input,
  plan_msgs__srv__MakeGlobalPlan_Request * output);

/// Initialize array of srv/MakeGlobalPlan messages.
/**
 * It allocates the memory for the number of elements and calls
 * plan_msgs__srv__MakeGlobalPlan_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__init(plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array, size_t size);

/// Finalize array of srv/MakeGlobalPlan messages.
/**
 * It calls
 * plan_msgs__srv__MakeGlobalPlan_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__fini(plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array);

/// Create array of srv/MakeGlobalPlan messages.
/**
 * It allocates the memory for the array and calls
 * plan_msgs__srv__MakeGlobalPlan_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
plan_msgs__srv__MakeGlobalPlan_Request__Sequence *
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__create(size_t size);

/// Destroy array of srv/MakeGlobalPlan messages.
/**
 * It calls
 * plan_msgs__srv__MakeGlobalPlan_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__destroy(plan_msgs__srv__MakeGlobalPlan_Request__Sequence * array);

/// Check for srv/MakeGlobalPlan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__are_equal(const plan_msgs__srv__MakeGlobalPlan_Request__Sequence * lhs, const plan_msgs__srv__MakeGlobalPlan_Request__Sequence * rhs);

/// Copy an array of srv/MakeGlobalPlan messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Request__Sequence__copy(
  const plan_msgs__srv__MakeGlobalPlan_Request__Sequence * input,
  plan_msgs__srv__MakeGlobalPlan_Request__Sequence * output);

/// Initialize srv/MakeGlobalPlan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * plan_msgs__srv__MakeGlobalPlan_Response
 * )) before or use
 * plan_msgs__srv__MakeGlobalPlan_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Response__init(plan_msgs__srv__MakeGlobalPlan_Response * msg);

/// Finalize srv/MakeGlobalPlan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Response__fini(plan_msgs__srv__MakeGlobalPlan_Response * msg);

/// Create srv/MakeGlobalPlan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * plan_msgs__srv__MakeGlobalPlan_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
plan_msgs__srv__MakeGlobalPlan_Response *
plan_msgs__srv__MakeGlobalPlan_Response__create();

/// Destroy srv/MakeGlobalPlan message.
/**
 * It calls
 * plan_msgs__srv__MakeGlobalPlan_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Response__destroy(plan_msgs__srv__MakeGlobalPlan_Response * msg);

/// Check for srv/MakeGlobalPlan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Response__are_equal(const plan_msgs__srv__MakeGlobalPlan_Response * lhs, const plan_msgs__srv__MakeGlobalPlan_Response * rhs);

/// Copy a srv/MakeGlobalPlan message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Response__copy(
  const plan_msgs__srv__MakeGlobalPlan_Response * input,
  plan_msgs__srv__MakeGlobalPlan_Response * output);

/// Initialize array of srv/MakeGlobalPlan messages.
/**
 * It allocates the memory for the number of elements and calls
 * plan_msgs__srv__MakeGlobalPlan_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__init(plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array, size_t size);

/// Finalize array of srv/MakeGlobalPlan messages.
/**
 * It calls
 * plan_msgs__srv__MakeGlobalPlan_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__fini(plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array);

/// Create array of srv/MakeGlobalPlan messages.
/**
 * It allocates the memory for the array and calls
 * plan_msgs__srv__MakeGlobalPlan_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
plan_msgs__srv__MakeGlobalPlan_Response__Sequence *
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__create(size_t size);

/// Destroy array of srv/MakeGlobalPlan messages.
/**
 * It calls
 * plan_msgs__srv__MakeGlobalPlan_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__destroy(plan_msgs__srv__MakeGlobalPlan_Response__Sequence * array);

/// Check for srv/MakeGlobalPlan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__are_equal(const plan_msgs__srv__MakeGlobalPlan_Response__Sequence * lhs, const plan_msgs__srv__MakeGlobalPlan_Response__Sequence * rhs);

/// Copy an array of srv/MakeGlobalPlan messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__srv__MakeGlobalPlan_Response__Sequence__copy(
  const plan_msgs__srv__MakeGlobalPlan_Response__Sequence * input,
  plan_msgs__srv__MakeGlobalPlan_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__SRV__DETAIL__MAKE_GLOBAL_PLAN__FUNCTIONS_H_
