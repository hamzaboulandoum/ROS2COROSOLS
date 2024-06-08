// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgePositionAngle.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgePositionAngle __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgePositionAngle __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HedgePositionAngle_
{
  using Type = HedgePositionAngle_<ContainerAllocator>;

  explicit HedgePositionAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address = 0;
      this->timestamp_ms = 0ll;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->z_m = 0.0;
      this->flags = 0;
      this->angle = 0.0;
    }
  }

  explicit HedgePositionAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address = 0;
      this->timestamp_ms = 0ll;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->z_m = 0.0;
      this->flags = 0;
      this->angle = 0.0;
    }
  }

  // field types and members
  using _address_type =
    uint8_t;
  _address_type address;
  using _timestamp_ms_type =
    int64_t;
  _timestamp_ms_type timestamp_ms;
  using _x_m_type =
    double;
  _x_m_type x_m;
  using _y_m_type =
    double;
  _y_m_type y_m;
  using _z_m_type =
    double;
  _z_m_type z_m;
  using _flags_type =
    uint8_t;
  _flags_type flags;
  using _angle_type =
    double;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__address(
    const uint8_t & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__timestamp_ms(
    const int64_t & _arg)
  {
    this->timestamp_ms = _arg;
    return *this;
  }
  Type & set__x_m(
    const double & _arg)
  {
    this->x_m = _arg;
    return *this;
  }
  Type & set__y_m(
    const double & _arg)
  {
    this->y_m = _arg;
    return *this;
  }
  Type & set__z_m(
    const double & _arg)
  {
    this->z_m = _arg;
    return *this;
  }
  Type & set__flags(
    const uint8_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__angle(
    const double & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgePositionAngle
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgePositionAngle
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgePositionAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HedgePositionAngle_ & other) const
  {
    if (this->address != other.address) {
      return false;
    }
    if (this->timestamp_ms != other.timestamp_ms) {
      return false;
    }
    if (this->x_m != other.x_m) {
      return false;
    }
    if (this->y_m != other.y_m) {
      return false;
    }
    if (this->z_m != other.z_m) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const HedgePositionAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HedgePositionAngle_

// alias to use template instance with default allocator
using HedgePositionAngle =
  marvelmind_ros2_msgs::msg::HedgePositionAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__STRUCT_HPP_
