// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/BeaconDistance.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__BeaconDistance __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__BeaconDistance __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BeaconDistance_
{
  using Type = BeaconDistance_<ContainerAllocator>;

  explicit BeaconDistance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address_hedge = 0;
      this->address_beacon = 0;
      this->distance_m = 0.0;
    }
  }

  explicit BeaconDistance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address_hedge = 0;
      this->address_beacon = 0;
      this->distance_m = 0.0;
    }
  }

  // field types and members
  using _address_hedge_type =
    uint8_t;
  _address_hedge_type address_hedge;
  using _address_beacon_type =
    uint8_t;
  _address_beacon_type address_beacon;
  using _distance_m_type =
    double;
  _distance_m_type distance_m;

  // setters for named parameter idiom
  Type & set__address_hedge(
    const uint8_t & _arg)
  {
    this->address_hedge = _arg;
    return *this;
  }
  Type & set__address_beacon(
    const uint8_t & _arg)
  {
    this->address_beacon = _arg;
    return *this;
  }
  Type & set__distance_m(
    const double & _arg)
  {
    this->distance_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__BeaconDistance
    std::shared_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__BeaconDistance
    std::shared_ptr<marvelmind_ros2_msgs::msg::BeaconDistance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BeaconDistance_ & other) const
  {
    if (this->address_hedge != other.address_hedge) {
      return false;
    }
    if (this->address_beacon != other.address_beacon) {
      return false;
    }
    if (this->distance_m != other.distance_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const BeaconDistance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BeaconDistance_

// alias to use template instance with default allocator
using BeaconDistance =
  marvelmind_ros2_msgs::msg::BeaconDistance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__STRUCT_HPP_
