// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/OpenSpace.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__OPEN_SPACE__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__OPEN_SPACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__OpenSpace __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__OpenSpace __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpenSpace_
{
  using Type = OpenSpace_<ContainerAllocator>;

  explicit OpenSpace_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->distance = 0.0f;
    }
  }

  explicit OpenSpace_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _angle_type =
    float;
  _angle_type angle;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::OpenSpace_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::OpenSpace_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::OpenSpace_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::OpenSpace_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__OpenSpace
    std::shared_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__OpenSpace
    std::shared_ptr<custom_msgs::msg::OpenSpace_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpenSpace_ & other) const
  {
    if (this->angle != other.angle) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpenSpace_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpenSpace_

// alias to use template instance with default allocator
using OpenSpace =
  custom_msgs::msg::OpenSpace_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__OPEN_SPACE__STRUCT_HPP_
