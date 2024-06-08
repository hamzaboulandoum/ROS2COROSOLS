// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindWaypoint __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindWaypoint __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarvelmindWaypoint_
{
  using Type = MarvelmindWaypoint_<ContainerAllocator>;

  explicit MarvelmindWaypoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_items = 0;
      this->item_index = 0;
      this->movement_type = 0;
      this->param1 = 0;
      this->param2 = 0;
      this->param3 = 0;
    }
  }

  explicit MarvelmindWaypoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_items = 0;
      this->item_index = 0;
      this->movement_type = 0;
      this->param1 = 0;
      this->param2 = 0;
      this->param3 = 0;
    }
  }

  // field types and members
  using _total_items_type =
    uint8_t;
  _total_items_type total_items;
  using _item_index_type =
    uint8_t;
  _item_index_type item_index;
  using _movement_type_type =
    uint8_t;
  _movement_type_type movement_type;
  using _param1_type =
    int16_t;
  _param1_type param1;
  using _param2_type =
    int16_t;
  _param2_type param2;
  using _param3_type =
    int16_t;
  _param3_type param3;

  // setters for named parameter idiom
  Type & set__total_items(
    const uint8_t & _arg)
  {
    this->total_items = _arg;
    return *this;
  }
  Type & set__item_index(
    const uint8_t & _arg)
  {
    this->item_index = _arg;
    return *this;
  }
  Type & set__movement_type(
    const uint8_t & _arg)
  {
    this->movement_type = _arg;
    return *this;
  }
  Type & set__param1(
    const int16_t & _arg)
  {
    this->param1 = _arg;
    return *this;
  }
  Type & set__param2(
    const int16_t & _arg)
  {
    this->param2 = _arg;
    return *this;
  }
  Type & set__param3(
    const int16_t & _arg)
  {
    this->param3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindWaypoint
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindWaypoint
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarvelmindWaypoint_ & other) const
  {
    if (this->total_items != other.total_items) {
      return false;
    }
    if (this->item_index != other.item_index) {
      return false;
    }
    if (this->movement_type != other.movement_type) {
      return false;
    }
    if (this->param1 != other.param1) {
      return false;
    }
    if (this->param2 != other.param2) {
      return false;
    }
    if (this->param3 != other.param3) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarvelmindWaypoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarvelmindWaypoint_

// alias to use template instance with default allocator
using MarvelmindWaypoint =
  marvelmind_ros2_msgs::msg::MarvelmindWaypoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__STRUCT_HPP_
