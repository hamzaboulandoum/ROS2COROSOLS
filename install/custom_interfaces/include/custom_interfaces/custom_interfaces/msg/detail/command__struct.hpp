// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__Command __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__Command __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Command_
{
  using Type = Command_<ContainerAllocator>;

  explicit Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0l;
      this->vy = 0l;
      this->vr = 0l;
      this->airbrush = 0l;
    }
  }

  explicit Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0l;
      this->vy = 0l;
      this->vr = 0l;
      this->airbrush = 0l;
    }
  }

  // field types and members
  using _vx_type =
    int32_t;
  _vx_type vx;
  using _vy_type =
    int32_t;
  _vy_type vy;
  using _vr_type =
    int32_t;
  _vr_type vr;
  using _airbrush_type =
    int32_t;
  _airbrush_type airbrush;

  // setters for named parameter idiom
  Type & set__vx(
    const int32_t & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const int32_t & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__vr(
    const int32_t & _arg)
  {
    this->vr = _arg;
    return *this;
  }
  Type & set__airbrush(
    const int32_t & _arg)
  {
    this->airbrush = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__Command
    std::shared_ptr<custom_interfaces::msg::Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__Command
    std::shared_ptr<custom_interfaces::msg::Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_ & other) const
  {
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->vr != other.vr) {
      return false;
    }
    if (this->airbrush != other.airbrush) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_

// alias to use template instance with default allocator
using Command =
  custom_interfaces::msg::Command_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_HPP_
