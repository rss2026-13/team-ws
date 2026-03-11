// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vs_msgs:msg/ConeLocationPixel.idl
// generated code does not contain a copyright notice

#ifndef VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__STRUCT_HPP_
#define VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vs_msgs__msg__ConeLocationPixel __attribute__((deprecated))
#else
# define DEPRECATED__vs_msgs__msg__ConeLocationPixel __declspec(deprecated)
#endif

namespace vs_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConeLocationPixel_
{
  using Type = ConeLocationPixel_<ContainerAllocator>;

  explicit ConeLocationPixel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
    }
  }

  explicit ConeLocationPixel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
    }
  }

  // field types and members
  using _u_type =
    float;
  _u_type u;
  using _v_type =
    float;
  _v_type v;

  // setters for named parameter idiom
  Type & set__u(
    const float & _arg)
  {
    this->u = _arg;
    return *this;
  }
  Type & set__v(
    const float & _arg)
  {
    this->v = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vs_msgs::msg::ConeLocationPixel_<ContainerAllocator> *;
  using ConstRawPtr =
    const vs_msgs::msg::ConeLocationPixel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vs_msgs::msg::ConeLocationPixel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vs_msgs::msg::ConeLocationPixel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vs_msgs__msg__ConeLocationPixel
    std::shared_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vs_msgs__msg__ConeLocationPixel
    std::shared_ptr<vs_msgs::msg::ConeLocationPixel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConeLocationPixel_ & other) const
  {
    if (this->u != other.u) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConeLocationPixel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConeLocationPixel_

// alias to use template instance with default allocator
using ConeLocationPixel =
  vs_msgs::msg::ConeLocationPixel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vs_msgs

#endif  // VS_MSGS__MSG__DETAIL__CONE_LOCATION_PIXEL__STRUCT_HPP_
