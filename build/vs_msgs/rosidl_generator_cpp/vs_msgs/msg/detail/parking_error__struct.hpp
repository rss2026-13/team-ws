// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vs_msgs:msg/ParkingError.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__PARKING_ERROR__STRUCT_HPP_
#define VS_MSGS__MSG__DETAIL__PARKING_ERROR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vs_msgs__msg__ParkingError __attribute__((deprecated))
#else
# define DEPRECATED__vs_msgs__msg__ParkingError __declspec(deprecated)
#endif

namespace vs_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ParkingError_
{
  using Type = ParkingError_<ContainerAllocator>;

  explicit ParkingError_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_error = 0.0f;
      this->y_error = 0.0f;
      this->distance_error = 0.0f;
    }
  }

  explicit ParkingError_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_error = 0.0f;
      this->y_error = 0.0f;
      this->distance_error = 0.0f;
    }
  }

  // field types and members
  using _x_error_type =
    float;
  _x_error_type x_error;
  using _y_error_type =
    float;
  _y_error_type y_error;
  using _distance_error_type =
    float;
  _distance_error_type distance_error;

  // setters for named parameter idiom
  Type & set__x_error(
    const float & _arg)
  {
    this->x_error = _arg;
    return *this;
  }
  Type & set__y_error(
    const float & _arg)
  {
    this->y_error = _arg;
    return *this;
  }
  Type & set__distance_error(
    const float & _arg)
  {
    this->distance_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vs_msgs::msg::ParkingError_<ContainerAllocator> *;
  using ConstRawPtr =
    const vs_msgs::msg::ParkingError_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vs_msgs::msg::ParkingError_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vs_msgs::msg::ParkingError_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vs_msgs__msg__ParkingError
    std::shared_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vs_msgs__msg__ParkingError
    std::shared_ptr<vs_msgs::msg::ParkingError_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParkingError_ & other) const
  {
    if (this->x_error != other.x_error) {
      return false;
    }
    if (this->y_error != other.y_error) {
      return false;
    }
    if (this->distance_error != other.distance_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParkingError_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParkingError_

// alias to use template instance with default allocator
using ParkingError =
  vs_msgs::msg::ParkingError_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vs_msgs

#endif  // VS_MSGS__MSG__DETAIL__PARKING_ERROR__STRUCT_HPP_
