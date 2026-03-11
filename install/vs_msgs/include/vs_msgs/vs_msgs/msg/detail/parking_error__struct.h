// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vs_msgs:msg/ParkingError.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__PARKING_ERROR__STRUCT_H_
#define VS_MSGS__MSG__DETAIL__PARKING_ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ParkingError in the package vs_msgs.
typedef struct vs_msgs__msg__ParkingError
{
  float x_error;
  float y_error;
  float distance_error;
} vs_msgs__msg__ParkingError;

// Struct for a sequence of vs_msgs__msg__ParkingError.
typedef struct vs_msgs__msg__ParkingError__Sequence
{
  vs_msgs__msg__ParkingError * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vs_msgs__msg__ParkingError__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VS_MSGS__MSG__DETAIL__PARKING_ERROR__STRUCT_H_
