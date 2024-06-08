// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuRaw.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuRaw __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuRaw __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HedgeImuRaw_
{
  using Type = HedgeImuRaw_<ContainerAllocator>;

  explicit HedgeImuRaw_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_ms = 0ll;
      this->acc_x = 0;
      this->acc_y = 0;
      this->acc_z = 0;
      this->gyro_x = 0;
      this->gyro_y = 0;
      this->gyro_z = 0;
      this->compass_x = 0;
      this->compass_y = 0;
      this->compass_z = 0;
    }
  }

  explicit HedgeImuRaw_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_ms = 0ll;
      this->acc_x = 0;
      this->acc_y = 0;
      this->acc_z = 0;
      this->gyro_x = 0;
      this->gyro_y = 0;
      this->gyro_z = 0;
      this->compass_x = 0;
      this->compass_y = 0;
      this->compass_z = 0;
    }
  }

  // field types and members
  using _timestamp_ms_type =
    int64_t;
  _timestamp_ms_type timestamp_ms;
  using _acc_x_type =
    int16_t;
  _acc_x_type acc_x;
  using _acc_y_type =
    int16_t;
  _acc_y_type acc_y;
  using _acc_z_type =
    int16_t;
  _acc_z_type acc_z;
  using _gyro_x_type =
    int16_t;
  _gyro_x_type gyro_x;
  using _gyro_y_type =
    int16_t;
  _gyro_y_type gyro_y;
  using _gyro_z_type =
    int16_t;
  _gyro_z_type gyro_z;
  using _compass_x_type =
    int16_t;
  _compass_x_type compass_x;
  using _compass_y_type =
    int16_t;
  _compass_y_type compass_y;
  using _compass_z_type =
    int16_t;
  _compass_z_type compass_z;

  // setters for named parameter idiom
  Type & set__timestamp_ms(
    const int64_t & _arg)
  {
    this->timestamp_ms = _arg;
    return *this;
  }
  Type & set__acc_x(
    const int16_t & _arg)
  {
    this->acc_x = _arg;
    return *this;
  }
  Type & set__acc_y(
    const int16_t & _arg)
  {
    this->acc_y = _arg;
    return *this;
  }
  Type & set__acc_z(
    const int16_t & _arg)
  {
    this->acc_z = _arg;
    return *this;
  }
  Type & set__gyro_x(
    const int16_t & _arg)
  {
    this->gyro_x = _arg;
    return *this;
  }
  Type & set__gyro_y(
    const int16_t & _arg)
  {
    this->gyro_y = _arg;
    return *this;
  }
  Type & set__gyro_z(
    const int16_t & _arg)
  {
    this->gyro_z = _arg;
    return *this;
  }
  Type & set__compass_x(
    const int16_t & _arg)
  {
    this->compass_x = _arg;
    return *this;
  }
  Type & set__compass_y(
    const int16_t & _arg)
  {
    this->compass_y = _arg;
    return *this;
  }
  Type & set__compass_z(
    const int16_t & _arg)
  {
    this->compass_z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuRaw
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuRaw
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuRaw_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HedgeImuRaw_ & other) const
  {
    if (this->timestamp_ms != other.timestamp_ms) {
      return false;
    }
    if (this->acc_x != other.acc_x) {
      return false;
    }
    if (this->acc_y != other.acc_y) {
      return false;
    }
    if (this->acc_z != other.acc_z) {
      return false;
    }
    if (this->gyro_x != other.gyro_x) {
      return false;
    }
    if (this->gyro_y != other.gyro_y) {
      return false;
    }
    if (this->gyro_z != other.gyro_z) {
      return false;
    }
    if (this->compass_x != other.compass_x) {
      return false;
    }
    if (this->compass_y != other.compass_y) {
      return false;
    }
    if (this->compass_z != other.compass_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const HedgeImuRaw_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HedgeImuRaw_

// alias to use template instance with default allocator
using HedgeImuRaw =
  marvelmind_ros2_msgs::msg::HedgeImuRaw_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__STRUCT_HPP_
