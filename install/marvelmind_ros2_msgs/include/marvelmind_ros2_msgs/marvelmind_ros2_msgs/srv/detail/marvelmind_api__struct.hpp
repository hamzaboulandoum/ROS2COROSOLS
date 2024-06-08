// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from marvelmind_ros2_msgs:srv/MarvelmindAPI.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__STRUCT_HPP_
#define MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Request __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Request __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MarvelmindAPI_Request_
{
  using Type = MarvelmindAPI_Request_<ContainerAllocator>;

  explicit MarvelmindAPI_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_id = 0ll;
    }
  }

  explicit MarvelmindAPI_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_id = 0ll;
    }
  }

  // field types and members
  using _command_id_type =
    int64_t;
  _command_id_type command_id;
  using _request_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__command_id(
    const int64_t & _arg)
  {
    this->command_id = _arg;
    return *this;
  }
  Type & set__request(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Request
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Request
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarvelmindAPI_Request_ & other) const
  {
    if (this->command_id != other.command_id) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarvelmindAPI_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarvelmindAPI_Request_

// alias to use template instance with default allocator
using MarvelmindAPI_Request =
  marvelmind_ros2_msgs::srv::MarvelmindAPI_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace marvelmind_ros2_msgs


#ifndef _WIN32
# define DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Response __attribute__((deprecated))
#else
# define DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Response __declspec(deprecated)
#endif

namespace marvelmind_ros2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MarvelmindAPI_Response_
{
  using Type = MarvelmindAPI_Response_<ContainerAllocator>;

  explicit MarvelmindAPI_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0l;
    }
  }

  explicit MarvelmindAPI_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _response_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__response(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Response
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__marvelmind_ros2_msgs__srv__MarvelmindAPI_Response
    std::shared_ptr<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarvelmindAPI_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarvelmindAPI_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarvelmindAPI_Response_

// alias to use template instance with default allocator
using MarvelmindAPI_Response =
  marvelmind_ros2_msgs::srv::MarvelmindAPI_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace marvelmind_ros2_msgs

namespace marvelmind_ros2_msgs
{

namespace srv
{

struct MarvelmindAPI
{
  using Request = marvelmind_ros2_msgs::srv::MarvelmindAPI_Request;
  using Response = marvelmind_ros2_msgs::srv::MarvelmindAPI_Response;
};

}  // namespace srv

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__STRUCT_HPP_
