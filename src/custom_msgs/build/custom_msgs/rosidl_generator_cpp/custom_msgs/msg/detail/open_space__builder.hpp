// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/OpenSpace.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__OPEN_SPACE__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__OPEN_SPACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/open_space__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_OpenSpace_distance
{
public:
  explicit Init_OpenSpace_distance(::custom_msgs::msg::OpenSpace & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::OpenSpace distance(::custom_msgs::msg::OpenSpace::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::OpenSpace msg_;
};

class Init_OpenSpace_angle
{
public:
  Init_OpenSpace_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenSpace_distance angle(::custom_msgs::msg::OpenSpace::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_OpenSpace_distance(msg_);
  }

private:
  ::custom_msgs::msg::OpenSpace msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::OpenSpace>()
{
  return custom_msgs::msg::builder::Init_OpenSpace_angle();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__OPEN_SPACE__BUILDER_HPP_
