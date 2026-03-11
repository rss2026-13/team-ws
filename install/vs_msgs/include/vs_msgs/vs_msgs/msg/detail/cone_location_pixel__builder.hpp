// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vs_msgs:msg/ConeLocationPixel.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__BUILDER_HPP_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vs_msgs/msg/detail/cone_location_pixel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vs_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeLocationPixel_v
{
public:
  explicit Init_ConeLocationPixel_v(::vs_msgs::msg::ConeLocationPixel & msg)
  : msg_(msg)
  {}
  ::vs_msgs::msg::ConeLocationPixel v(::vs_msgs::msg::ConeLocationPixel::_v_type arg)
  {
    msg_.v = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vs_msgs::msg::ConeLocationPixel msg_;
};

class Init_ConeLocationPixel_u
{
public:
  Init_ConeLocationPixel_u()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeLocationPixel_v u(::vs_msgs::msg::ConeLocationPixel::_u_type arg)
  {
    msg_.u = std::move(arg);
    return Init_ConeLocationPixel_v(msg_);
  }

private:
  ::vs_msgs::msg::ConeLocationPixel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vs_msgs::msg::ConeLocationPixel>()
{
  return vs_msgs::msg::builder::Init_ConeLocationPixel_u();
}

}  // namespace vs_msgs

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__BUILDER_HPP_
