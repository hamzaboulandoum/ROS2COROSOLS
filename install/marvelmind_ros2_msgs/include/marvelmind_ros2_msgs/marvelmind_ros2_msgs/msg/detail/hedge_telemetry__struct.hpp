// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeTelemetry.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeTelemetry __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeTelemetry __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HedgeTelemetry_
{
  using Type = HedgeTelemetry_<ContainerAllocator>;

  explicit HedgeTelemetry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0;
      this->rssi_dbm = 0;
    }
  }

  explicit HedgeTelemetry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0;
      this->rssi_dbm = 0;
    }
  }

  // field types and members
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _rssi_dbm_type =
    int8_t;
  _rssi_dbm_type rssi_dbm;

  // setters for named parameter idiom
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__rssi_dbm(
    const int8_t & _arg)
  {
    this->rssi_dbm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeTelemetry
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeTelemetry
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeTelemetry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HedgeTelemetry_ & other) const
  {
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->rssi_dbm != other.rssi_dbm) {
      return false;
    }
    return true;
  }
  bool operator!=(const HedgeTelemetry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HedgeTelemetry_

// alias to use template instance with default allocator
using HedgeTelemetry =
  marvelmind_ros2_msgs::msg::HedgeTelemetry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__STRUCT_HPP_
