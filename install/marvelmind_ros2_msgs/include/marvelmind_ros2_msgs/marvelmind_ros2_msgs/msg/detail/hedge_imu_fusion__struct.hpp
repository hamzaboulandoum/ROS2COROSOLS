// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuFusion.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_FUSION__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_FUSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuFusion __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuFusion __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HedgeImuFusion_
{
  using Type = HedgeImuFusion_<ContainerAllocator>;

  explicit HedgeImuFusion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_ms = 0ll;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->z_m = 0.0;
      this->qw = 0.0;
      this->qx = 0.0;
      this->qy = 0.0;
      this->qz = 0.0;
      this->vx = 0.0;
      this->vy = 0.0;
      this->vz = 0.0;
      this->ax = 0.0;
      this->ay = 0.0;
      this->az = 0.0;
    }
  }

  explicit HedgeImuFusion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_ms = 0ll;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->z_m = 0.0;
      this->qw = 0.0;
      this->qx = 0.0;
      this->qy = 0.0;
      this->qz = 0.0;
      this->vx = 0.0;
      this->vy = 0.0;
      this->vz = 0.0;
      this->ax = 0.0;
      this->ay = 0.0;
      this->az = 0.0;
    }
  }

  // field types and members
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
  using _qw_type =
    double;
  _qw_type qw;
  using _qx_type =
    double;
  _qx_type qx;
  using _qy_type =
    double;
  _qy_type qy;
  using _qz_type =
    double;
  _qz_type qz;
  using _vx_type =
    double;
  _vx_type vx;
  using _vy_type =
    double;
  _vy_type vy;
  using _vz_type =
    double;
  _vz_type vz;
  using _ax_type =
    double;
  _ax_type ax;
  using _ay_type =
    double;
  _ay_type ay;
  using _az_type =
    double;
  _az_type az;

  // setters for named parameter idiom
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
  Type & set__qw(
    const double & _arg)
  {
    this->qw = _arg;
    return *this;
  }
  Type & set__qx(
    const double & _arg)
  {
    this->qx = _arg;
    return *this;
  }
  Type & set__qy(
    const double & _arg)
  {
    this->qy = _arg;
    return *this;
  }
  Type & set__qz(
    const double & _arg)
  {
    this->qz = _arg;
    return *this;
  }
  Type & set__vx(
    const double & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const double & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__vz(
    const double & _arg)
  {
    this->vz = _arg;
    return *this;
  }
  Type & set__ax(
    const double & _arg)
  {
    this->ax = _arg;
    return *this;
  }
  Type & set__ay(
    const double & _arg)
  {
    this->ay = _arg;
    return *this;
  }
  Type & set__az(
    const double & _arg)
  {
    this->az = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuFusion
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__HedgeImuFusion
    std::shared_ptr<marvelmind_ros2_msgs::msg::HedgeImuFusion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HedgeImuFusion_ & other) const
  {
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
    if (this->qw != other.qw) {
      return false;
    }
    if (this->qx != other.qx) {
      return false;
    }
    if (this->qy != other.qy) {
      return false;
    }
    if (this->qz != other.qz) {
      return false;
    }
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->vz != other.vz) {
      return false;
    }
    if (this->ax != other.ax) {
      return false;
    }
    if (this->ay != other.ay) {
      return false;
    }
    if (this->az != other.az) {
      return false;
    }
    return true;
  }
  bool operator!=(const HedgeImuFusion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HedgeImuFusion_

// alias to use template instance with default allocator
using HedgeImuFusion =
  marvelmind_ros2_msgs::msg::HedgeImuFusion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_FUSION__STRUCT_HPP_
