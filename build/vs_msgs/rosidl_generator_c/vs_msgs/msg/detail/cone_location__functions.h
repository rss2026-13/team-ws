// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vs_msgs:msg/ConeLocation.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION__FUNCTIONS_H_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "vs_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "vs_msgs/msg/detail/cone_location__struct.h"

/// Initialize msg/ConeLocation message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vs_msgs__msg__ConeLocation
 * )) before or use
 * vs_msgs__msg__ConeLocation__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
bool
vs_msgs__msg__ConeLocation__init(vs_msgs__msg__ConeLocation * msg);

/// Finalize msg/ConeLocation message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
void
vs_msgs__msg__ConeLocation__fini(vs_msgs__msg__ConeLocation * msg);

/// Create msg/ConeLocation message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vs_msgs__msg__ConeLocation__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
vs_msgs__msg__ConeLocation *
vs_msgs__msg__ConeLocation__create();

/// Destroy msg/ConeLocation message.
/**
 * It calls
 * vs_msgs__msg__ConeLocation__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
void
vs_msgs__msg__ConeLocation__destroy(vs_msgs__msg__ConeLocation * msg);

/// Check for msg/ConeLocation message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
bool
vs_msgs__msg__ConeLocation__are_equal(const vs_msgs__msg__ConeLocation * lhs, const vs_msgs__msg__ConeLocation * rhs);

/// Copy a msg/ConeLocation message.
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
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
bool
vs_msgs__msg__ConeLocation__copy(
  const vs_msgs__msg__ConeLocation * input,
  vs_msgs__msg__ConeLocation * output);

/// Initialize array of msg/ConeLocation messages.
/**
 * It allocates the memory for the number of elements and calls
 * vs_msgs__msg__ConeLocation__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
bool
vs_msgs__msg__ConeLocation__Sequence__init(vs_msgs__msg__ConeLocation__Sequence * array, size_t size);

/// Finalize array of msg/ConeLocation messages.
/**
 * It calls
 * vs_msgs__msg__ConeLocation__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
void
vs_msgs__msg__ConeLocation__Sequence__fini(vs_msgs__msg__ConeLocation__Sequence * array);

/// Create array of msg/ConeLocation messages.
/**
 * It allocates the memory for the array and calls
 * vs_msgs__msg__ConeLocation__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
vs_msgs__msg__ConeLocation__Sequence *
vs_msgs__msg__ConeLocation__Sequence__create(size_t size);

/// Destroy array of msg/ConeLocation messages.
/**
 * It calls
 * vs_msgs__msg__ConeLocation__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
void
vs_msgs__msg__ConeLocation__Sequence__destroy(vs_msgs__msg__ConeLocation__Sequence * array);

/// Check for msg/ConeLocation message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
bool
vs_msgs__msg__ConeLocation__Sequence__are_equal(const vs_msgs__msg__ConeLocation__Sequence * lhs, const vs_msgs__msg__ConeLocation__Sequence * rhs);

/// Copy an array of msg/ConeLocation messages.
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
ROSIDL_GENERATOR_C_PUBLIC_vs_msgs
bool
vs_msgs__msg__ConeLocation__Sequence__copy(
  const vs_msgs__msg__ConeLocation__Sequence * input,
  vs_msgs__msg__ConeLocation__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION__FUNCTIONS_H_
