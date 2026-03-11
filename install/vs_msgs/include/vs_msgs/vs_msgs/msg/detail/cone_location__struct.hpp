// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vs_msgs:msg/ConeLocation.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION__STRUCT_HPP_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vs_msgs__msg__ConeLocation __attribute__((deprecated))
#else
# define DEPRECATED__vs_msgs__msg__ConeLocation __declspec(deprecated)
#endif

namespace vs_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConeLocation_
{
  using Type = ConeLocation_<ContainerAllocator>;

  explicit ConeLocation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_pos = 0.0f;
      this->y_pos = 0.0f;
    }
  }

  explicit ConeLocation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_pos = 0.0f;
      this->y_pos = 0.0f;
    }
  }

  // field types and members
  using _x_pos_type =
    float;
  _x_pos_type x_pos;
  using _y_pos_type =
    float;
  _y_pos_type y_pos;

  // setters for named parameter idiom
  Type & set__x_pos(
    const float & _arg)
  {
    this->x_pos = _arg;
    return *this;
  }
  Type & set__y_pos(
    const float & _arg)
  {
    this->y_pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vs_msgs::msg::ConeLocation_<ContainerAllocator> *;
  using ConstRawPtr =
    const vs_msgs::msg::ConeLocation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vs_msgs::msg::ConeLocation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vs_msgs::msg::ConeLocation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vs_msgs__msg__ConeLocation
    std::shared_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vs_msgs__msg__ConeLocation
    std::shared_ptr<vs_msgs::msg::ConeLocation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConeLocation_ & other) const
  {
    if (this->x_pos != other.x_pos) {
      return false;
    }
    if (this->y_pos != other.y_pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConeLocation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConeLocation_

// alias to use template instance with default allocator
using ConeLocation =
  vs_msgs::msg::ConeLocation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vs_msgs

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION__STRUCT_HPP_
