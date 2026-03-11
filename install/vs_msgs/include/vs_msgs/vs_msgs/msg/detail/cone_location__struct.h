// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vs_msgs:msg/ConeLocation.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION__STRUCT_H_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ConeLocation in the package vs_msgs.
typedef struct vs_msgs__msg__ConeLocation
{
  float x_pos;
  float y_pos;
} vs_msgs__msg__ConeLocation;

// Struct for a sequence of vs_msgs__msg__ConeLocation.
typedef struct vs_msgs__msg__ConeLocation__Sequence
{
  vs_msgs__msg__ConeLocation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vs_msgs__msg__ConeLocation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION__STRUCT_H_
