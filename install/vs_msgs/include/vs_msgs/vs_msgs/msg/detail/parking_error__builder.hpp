// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vs_msgs:msg/ParkingError.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__PARKING_ERROR__BUILDER_HPP_
#define VS_MSGS__MSG__DETAIL__PARKING_ERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vs_msgs/msg/detail/parking_error__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vs_msgs
{

namespace msg
{

namespace builder
{

class Init_ParkingError_distance_error
{
public:
  explicit Init_ParkingError_distance_error(::vs_msgs::msg::ParkingError & msg)
  : msg_(msg)
  {}
  ::vs_msgs::msg::ParkingError distance_error(::vs_msgs::msg::ParkingError::_distance_error_type arg)
  {
    msg_.distance_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vs_msgs::msg::ParkingError msg_;
};

class Init_ParkingError_y_error
{
public:
  explicit Init_ParkingError_y_error(::vs_msgs::msg::ParkingError & msg)
  : msg_(msg)
  {}
  Init_ParkingError_distance_error y_error(::vs_msgs::msg::ParkingError::_y_error_type arg)
  {
    msg_.y_error = std::move(arg);
    return Init_ParkingError_distance_error(msg_);
  }

private:
  ::vs_msgs::msg::ParkingError msg_;
};

class Init_ParkingError_x_error
{
public:
  Init_ParkingError_x_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParkingError_y_error x_error(::vs_msgs::msg::ParkingError::_x_error_type arg)
  {
    msg_.x_error = std::move(arg);
    return Init_ParkingError_y_error(msg_);
  }

private:
  ::vs_msgs::msg::ParkingError msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vs_msgs::msg::ParkingError>()
{
  return vs_msgs::msg::builder::Init_ParkingError_x_error();
}

}  // namespace vs_msgs

#endif  // VS_MSGS__MSG__DETAIL__PARKING_ERROR__BUILDER_HPP_
