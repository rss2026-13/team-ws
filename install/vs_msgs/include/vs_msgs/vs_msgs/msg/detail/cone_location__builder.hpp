// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vs_msgs:msg/ConeLocation.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION__BUILDER_HPP_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vs_msgs/msg/detail/cone_location__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vs_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeLocation_y_pos
{
public:
  explicit Init_ConeLocation_y_pos(::vs_msgs::msg::ConeLocation & msg)
  : msg_(msg)
  {}
  ::vs_msgs::msg::ConeLocation y_pos(::vs_msgs::msg::ConeLocation::_y_pos_type arg)
  {
    msg_.y_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vs_msgs::msg::ConeLocation msg_;
};

class Init_ConeLocation_x_pos
{
public:
  Init_ConeLocation_x_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeLocation_y_pos x_pos(::vs_msgs::msg::ConeLocation::_x_pos_type arg)
  {
    msg_.x_pos = std::move(arg);
    return Init_ConeLocation_y_pos(msg_);
  }

private:
  ::vs_msgs::msg::ConeLocation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vs_msgs::msg::ConeLocation>()
{
  return vs_msgs::msg::builder::Init_ConeLocation_x_pos();
}

}  // namespace vs_msgs

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION__BUILDER_HPP_
