// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindUserData.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindUserData __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindUserData __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarvelmindUserData_
{
  using Type = MarvelmindUserData_<ContainerAllocator>;

  explicit MarvelmindUserData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_ms = 0ll;
    }
  }

  explicit MarvelmindUserData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_ms = 0ll;
    }
  }

  // field types and members
  using _timestamp_ms_type =
    int64_t;
  _timestamp_ms_type timestamp_ms;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__timestamp_ms(
    const int64_t & _arg)
  {
    this->timestamp_ms = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindUserData
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__msg__MarvelmindUserData
    std::shared_ptr<marvelmind_ros2_msgs::msg::MarvelmindUserData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarvelmindUserData_ & other) const
  {
    if (this->timestamp_ms != other.timestamp_ms) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarvelmindUserData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarvelmindUserData_

// alias to use template instance with default allocator
using MarvelmindUserData =
  marvelmind_ros2_msgs::msg::MarvelmindUserData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__STRUCT_HPP_
