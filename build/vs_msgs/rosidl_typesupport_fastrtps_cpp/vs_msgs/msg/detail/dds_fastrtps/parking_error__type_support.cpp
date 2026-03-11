// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vs_msgs:msg/ParkingError.idl
// generated code does not contain a copyright notice
#include "vs_msgs/msg/detail/parking_error__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vs_msgs/msg/detail/parking_error__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vs_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vs_msgs
cdr_serialize(
  const vs_msgs::msg::ParkingError & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: x_error
  cdr << ros_message.x_error;
  // Member: y_error
  cdr << ros_message.y_error;
  // Member: distance_error
  cdr << ros_message.distance_error;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vs_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vs_msgs::msg::ParkingError & ros_message)
{
  // Member: x_error
  cdr >> ros_message.x_error;

  // Member: y_error
  cdr >> ros_message.y_error;

  // Member: distance_error
  cdr >> ros_message.distance_error;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vs_msgs
get_serialized_size(
  const vs_msgs::msg::ParkingError & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: x_error
  {
    size_t item_size = sizeof(ros_message.x_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_error
  {
    size_t item_size = sizeof(ros_message.y_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance_error
  {
    size_t item_size = sizeof(ros_message.distance_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vs_msgs
max_serialized_size_ParkingError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: x_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: y_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vs_msgs::msg::ParkingError;
    is_plain =
      (
      offsetof(DataType, distance_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ParkingError__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vs_msgs::msg::ParkingError *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ParkingError__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vs_msgs::msg::ParkingError *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ParkingError__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vs_msgs::msg::ParkingError *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ParkingError__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ParkingError(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ParkingError__callbacks = {
  "vs_msgs::msg",
  "ParkingError",
  _ParkingError__cdr_serialize,
  _ParkingError__cdr_deserialize,
  _ParkingError__get_serialized_size,
  _ParkingError__max_serialized_size
};

static rosidl_message_type_support_t _ParkingError__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ParkingError__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vs_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vs_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vs_msgs::msg::ParkingError>()
{
  return &vs_msgs::msg::typesupport_fastrtps_cpp::_ParkingError__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vs_msgs, msg, ParkingError)() {
  return &vs_msgs::msg::typesupport_fastrtps_cpp::_ParkingError__handle;
}

#ifdef __cplusplus
}
#endif
