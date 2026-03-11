// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vs_msgs:msg/ParkingError.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__PARKING_ERROR__TRAITS_HPP_
#define VS_MSGS__MSG__DETAIL__PARKING_ERROR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vs_msgs/msg/detail/parking_error__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vs_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ParkingError & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_error
  {
    out << "x_error: ";
    rosidl_generator_traits::value_to_yaml(msg.x_error, out);
    out << ", ";
  }

  // member: y_error
  {
    out << "y_error: ";
    rosidl_generator_traits::value_to_yaml(msg.y_error, out);
    out << ", ";
  }

  // member: distance_error
  {
    out << "distance_error: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParkingError & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_error: ";
    rosidl_generator_traits::value_to_yaml(msg.x_error, out);
    out << "\n";
  }

  // member: y_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_error: ";
    rosidl_generator_traits::value_to_yaml(msg.y_error, out);
    out << "\n";
  }

  // member: distance_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_error: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParkingError & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vs_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vs_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vs_msgs::msg::ParkingError & msg,
  std::ostream & out, size_t indentation = 0)
{
  vs_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vs_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vs_msgs::msg::ParkingError & msg)
{
  return vs_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vs_msgs::msg::ParkingError>()
{
  return "vs_msgs::msg::ParkingError";
}

template<>
inline const char * name<vs_msgs::msg::ParkingError>()
{
  return "vs_msgs/msg/ParkingError";
}

template<>
struct has_fixed_size<vs_msgs::msg::ParkingError>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vs_msgs::msg::ParkingError>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vs_msgs::msg::ParkingError>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VS_MSGS__MSG__DETAIL__PARKING_ERROR__TRAITS_HPP_
