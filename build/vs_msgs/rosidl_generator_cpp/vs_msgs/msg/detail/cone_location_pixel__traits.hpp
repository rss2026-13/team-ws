// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vs_msgs:msg/ConeLocationPixel.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__TRAITS_HPP_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vs_msgs/msg/detail/cone_location_pixel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vs_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ConeLocationPixel & msg,
  std::ostream & out)
{
  out << "{";
  // member: u
  {
    out << "u: ";
    rosidl_generator_traits::value_to_yaml(msg.u, out);
    out << ", ";
  }

  // member: v
  {
    out << "v: ";
    rosidl_generator_traits::value_to_yaml(msg.v, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConeLocationPixel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: u
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "u: ";
    rosidl_generator_traits::value_to_yaml(msg.u, out);
    out << "\n";
  }

  // member: v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v: ";
    rosidl_generator_traits::value_to_yaml(msg.v, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConeLocationPixel & msg, bool use_flow_style = false)
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
  const vs_msgs::msg::ConeLocationPixel & msg,
  std::ostream & out, size_t indentation = 0)
{
  vs_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vs_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vs_msgs::msg::ConeLocationPixel & msg)
{
  return vs_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vs_msgs::msg::ConeLocationPixel>()
{
  return "vs_msgs::msg::ConeLocationPixel";
}

template<>
inline const char * name<vs_msgs::msg::ConeLocationPixel>()
{
  return "vs_msgs/msg/ConeLocationPixel";
}

template<>
struct has_fixed_size<vs_msgs::msg::ConeLocationPixel>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vs_msgs::msg::ConeLocationPixel>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vs_msgs::msg::ConeLocationPixel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__TRAITS_HPP_
