// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/Commands.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__Commands_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__Commands_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Commands_Request_
{
  using Type = Commands_Request_<ContainerAllocator>;

  explicit Commands_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->vr = 0.0;
      this->stepperx = 0.0;
      this->steppery = 0.0;
      this->airbrush = 0l;
      this->ab_servo = 0l;
    }
  }

  explicit Commands_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->vr = 0.0;
      this->stepperx = 0.0;
      this->steppery = 0.0;
      this->airbrush = 0l;
      this->ab_servo = 0l;
    }
  }

  // field types and members
  using _vx_type =
    double;
  _vx_type vx;
  using _vy_type =
    double;
  _vy_type vy;
  using _vr_type =
    double;
  _vr_type vr;
  using _stepperx_type =
    double;
  _stepperx_type stepperx;
  using _steppery_type =
    double;
  _steppery_type steppery;
  using _airbrush_type =
    int32_t;
  _airbrush_type airbrush;
  using _ab_servo_type =
    int32_t;
  _ab_servo_type ab_servo;

  // setters for named parameter idiom
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
  Type & set__vr(
    const double & _arg)
  {
    this->vr = _arg;
    return *this;
  }
  Type & set__stepperx(
    const double & _arg)
  {
    this->stepperx = _arg;
    return *this;
  }
  Type & set__steppery(
    const double & _arg)
  {
    this->steppery = _arg;
    return *this;
  }
  Type & set__airbrush(
    const int32_t & _arg)
  {
    this->airbrush = _arg;
    return *this;
  }
  Type & set__ab_servo(
    const int32_t & _arg)
  {
    this->ab_servo = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::Commands_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::Commands_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Commands_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Commands_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__Commands_Request
    std::shared_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__Commands_Request
    std::shared_ptr<custom_interfaces::srv::Commands_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Commands_Request_ & other) const
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
    if (this->stepperx != other.stepperx) {
      return false;
    }
    if (this->steppery != other.steppery) {
      return false;
    }
    if (this->airbrush != other.airbrush) {
      return false;
    }
    if (this->ab_servo != other.ab_servo) {
      return false;
    }
    return true;
  }
  bool operator!=(const Commands_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Commands_Request_

// alias to use template instance with default allocator
using Commands_Request =
  custom_interfaces::srv::Commands_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__Commands_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__Commands_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Commands_Response_
{
  using Type = Commands_Response_<ContainerAllocator>;

  explicit Commands_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = "";
    }
  }

  explicit Commands_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : outcome(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = "";
    }
  }

  // field types and members
  using _outcome_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _outcome_type outcome;

  // setters for named parameter idiom
  Type & set__outcome(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->outcome = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::Commands_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::Commands_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Commands_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Commands_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__Commands_Response
    std::shared_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__Commands_Response
    std::shared_ptr<custom_interfaces::srv::Commands_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Commands_Response_ & other) const
  {
    if (this->outcome != other.outcome) {
      return false;
    }
    return true;
  }
  bool operator!=(const Commands_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Commands_Response_

// alias to use template instance with default allocator
using Commands_Response =
  custom_interfaces::srv::Commands_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct Commands
{
  using Request = custom_interfaces::srv::Commands_Request;
  using Response = custom_interfaces::srv::Commands_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__STRUCT_HPP_
