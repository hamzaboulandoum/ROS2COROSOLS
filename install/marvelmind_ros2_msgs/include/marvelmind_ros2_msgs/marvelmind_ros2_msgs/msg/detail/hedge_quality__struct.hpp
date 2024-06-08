// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeQuality.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_QUALITY__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_QUALITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeQuality __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeQuality __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HedgeQuality_
{
  using Type = HedgeQuality_<ContainerAllocator>;

  explicit HedgeQuality_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address = 0;
      this->quality_percents = 0;
    }
  }

  explicit HedgeQuality_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address = 0;
      this->quality_percents = 0;
    }
  }

  // field types and members
  using _address_type =
    uint8_t;
  _address_type address;
  using _quality_percents_type =
    uint8_t;
  _quality_percents_type quality_percents;

  // setters for named parameter idiom
  Type & set__address(
    const uint8_t & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__quality_percents(
    const uint8_t & _arg)
  {
    this->quality_percents = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeQuality
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeQuality
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeQuality_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HedgeQuality_ & other) const
  {
    if (this->address != other.address) {
      return false;
    }
    if (this->quality_percents != other.quality_percents) {
      return false;
    }
    return true;
  }
  bool operator!=(const HedgeQuality_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HedgeQuality_

// alias to use template instance with default allocator
using HedgeQuality =
  marvelmind_ros2_msgs::msg::HedgeQuality_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_QUALITY__STRUCT_HPP_
