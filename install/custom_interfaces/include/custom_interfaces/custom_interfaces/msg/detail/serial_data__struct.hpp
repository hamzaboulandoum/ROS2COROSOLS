// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__SerialData __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__SerialData __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SerialData_
{
  using Type = SerialData_<ContainerAllocator>;

  explicit SerialData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_speed = 0.0;
      this->y_speed = 0.0;
      this->z_speed = 0.0;
      this->x_accel = 0.0;
      this->y_accel = 0.0;
      this->z_accel = 0.0;
      this->heading = 0.0;
      this->pitch = 0.0;
      this->roll = 0.0;
      this->power_voltage = 0.0;
      this->stepper_x = 0.0;
      this->stepper_y = 0.0;
    }
  }

  explicit SerialData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_speed = 0.0;
      this->y_speed = 0.0;
      this->z_speed = 0.0;
      this->x_accel = 0.0;
      this->y_accel = 0.0;
      this->z_accel = 0.0;
      this->heading = 0.0;
      this->pitch = 0.0;
      this->roll = 0.0;
      this->power_voltage = 0.0;
      this->stepper_x = 0.0;
      this->stepper_y = 0.0;
    }
  }

  // field types and members
  using _x_speed_type =
    double;
  _x_speed_type x_speed;
  using _y_speed_type =
    double;
  _y_speed_type y_speed;
  using _z_speed_type =
    double;
  _z_speed_type z_speed;
  using _x_accel_type =
    double;
  _x_accel_type x_accel;
  using _y_accel_type =
    double;
  _y_accel_type y_accel;
  using _z_accel_type =
    double;
  _z_accel_type z_accel;
  using _heading_type =
    double;
  _heading_type heading;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _roll_type =
    double;
  _roll_type roll;
  using _power_voltage_type =
    double;
  _power_voltage_type power_voltage;
  using _stepper_x_type =
    double;
  _stepper_x_type stepper_x;
  using _stepper_y_type =
    double;
  _stepper_y_type stepper_y;

  // setters for named parameter idiom
  Type & set__x_speed(
    const double & _arg)
  {
    this->x_speed = _arg;
    return *this;
  }
  Type & set__y_speed(
    const double & _arg)
  {
    this->y_speed = _arg;
    return *this;
  }
  Type & set__z_speed(
    const double & _arg)
  {
    this->z_speed = _arg;
    return *this;
  }
  Type & set__x_accel(
    const double & _arg)
  {
    this->x_accel = _arg;
    return *this;
  }
  Type & set__y_accel(
    const double & _arg)
  {
    this->y_accel = _arg;
    return *this;
  }
  Type & set__z_accel(
    const double & _arg)
  {
    this->z_accel = _arg;
    return *this;
  }
  Type & set__heading(
    const double & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__power_voltage(
    const double & _arg)
  {
    this->power_voltage = _arg;
    return *this;
  }
  Type & set__stepper_x(
    const double & _arg)
  {
    this->stepper_x = _arg;
    return *this;
  }
  Type & set__stepper_y(
    const double & _arg)
  {
    this->stepper_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::SerialData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::SerialData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::SerialData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::SerialData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__SerialData
    std::shared_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__SerialData
    std::shared_ptr<custom_interfaces::msg::SerialData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SerialData_ & other) const
  {
    if (this->x_speed != other.x_speed) {
      return false;
    }
    if (this->y_speed != other.y_speed) {
      return false;
    }
    if (this->z_speed != other.z_speed) {
      return false;
    }
    if (this->x_accel != other.x_accel) {
      return false;
    }
    if (this->y_accel != other.y_accel) {
      return false;
    }
    if (this->z_accel != other.z_accel) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->power_voltage != other.power_voltage) {
      return false;
    }
    if (this->stepper_x != other.stepper_x) {
      return false;
    }
    if (this->stepper_y != other.stepper_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const SerialData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SerialData_

// alias to use template instance with default allocator
using SerialData =
  custom_interfaces::msg::SerialData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_
