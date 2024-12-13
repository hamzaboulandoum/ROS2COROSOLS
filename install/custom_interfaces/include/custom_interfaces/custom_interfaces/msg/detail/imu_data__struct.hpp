// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__ImuData __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__ImuData __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuData_
{
  using Type = ImuData_<ContainerAllocator>;

  explicit ImuData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gyroscope_x = 0.0f;
      this->gyroscope_y = 0.0f;
      this->gyroscope_z = 0.0f;
      this->accelerometer_x = 0.0f;
      this->accelerometer_y = 0.0f;
      this->accelerometer_z = 0.0f;
      this->magnetometer_x = 0.0f;
      this->magnetometer_y = 0.0f;
      this->magnetometer_z = 0.0f;
      this->roll_speed = 0.0f;
      this->pitch_speed = 0.0f;
      this->heading_speed = 0.0f;
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->heading = 0.0f;
      this->q1 = 0.0f;
      this->q2 = 0.0f;
      this->q3 = 0.0f;
      this->q4 = 0.0f;
      this->temperature = 0.0f;
      this->pressure = 0.0f;
      this->pressure_temp = 0.0f;
      this->timestamp = 0l;
    }
  }

  explicit ImuData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gyroscope_x = 0.0f;
      this->gyroscope_y = 0.0f;
      this->gyroscope_z = 0.0f;
      this->accelerometer_x = 0.0f;
      this->accelerometer_y = 0.0f;
      this->accelerometer_z = 0.0f;
      this->magnetometer_x = 0.0f;
      this->magnetometer_y = 0.0f;
      this->magnetometer_z = 0.0f;
      this->roll_speed = 0.0f;
      this->pitch_speed = 0.0f;
      this->heading_speed = 0.0f;
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->heading = 0.0f;
      this->q1 = 0.0f;
      this->q2 = 0.0f;
      this->q3 = 0.0f;
      this->q4 = 0.0f;
      this->temperature = 0.0f;
      this->pressure = 0.0f;
      this->pressure_temp = 0.0f;
      this->timestamp = 0l;
    }
  }

  // field types and members
  using _gyroscope_x_type =
    float;
  _gyroscope_x_type gyroscope_x;
  using _gyroscope_y_type =
    float;
  _gyroscope_y_type gyroscope_y;
  using _gyroscope_z_type =
    float;
  _gyroscope_z_type gyroscope_z;
  using _accelerometer_x_type =
    float;
  _accelerometer_x_type accelerometer_x;
  using _accelerometer_y_type =
    float;
  _accelerometer_y_type accelerometer_y;
  using _accelerometer_z_type =
    float;
  _accelerometer_z_type accelerometer_z;
  using _magnetometer_x_type =
    float;
  _magnetometer_x_type magnetometer_x;
  using _magnetometer_y_type =
    float;
  _magnetometer_y_type magnetometer_y;
  using _magnetometer_z_type =
    float;
  _magnetometer_z_type magnetometer_z;
  using _roll_speed_type =
    float;
  _roll_speed_type roll_speed;
  using _pitch_speed_type =
    float;
  _pitch_speed_type pitch_speed;
  using _heading_speed_type =
    float;
  _heading_speed_type heading_speed;
  using _roll_type =
    float;
  _roll_type roll;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _heading_type =
    float;
  _heading_type heading;
  using _q1_type =
    float;
  _q1_type q1;
  using _q2_type =
    float;
  _q2_type q2;
  using _q3_type =
    float;
  _q3_type q3;
  using _q4_type =
    float;
  _q4_type q4;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _pressure_type =
    float;
  _pressure_type pressure;
  using _pressure_temp_type =
    float;
  _pressure_temp_type pressure_temp;
  using _timestamp_type =
    int32_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__gyroscope_x(
    const float & _arg)
  {
    this->gyroscope_x = _arg;
    return *this;
  }
  Type & set__gyroscope_y(
    const float & _arg)
  {
    this->gyroscope_y = _arg;
    return *this;
  }
  Type & set__gyroscope_z(
    const float & _arg)
  {
    this->gyroscope_z = _arg;
    return *this;
  }
  Type & set__accelerometer_x(
    const float & _arg)
  {
    this->accelerometer_x = _arg;
    return *this;
  }
  Type & set__accelerometer_y(
    const float & _arg)
  {
    this->accelerometer_y = _arg;
    return *this;
  }
  Type & set__accelerometer_z(
    const float & _arg)
  {
    this->accelerometer_z = _arg;
    return *this;
  }
  Type & set__magnetometer_x(
    const float & _arg)
  {
    this->magnetometer_x = _arg;
    return *this;
  }
  Type & set__magnetometer_y(
    const float & _arg)
  {
    this->magnetometer_y = _arg;
    return *this;
  }
  Type & set__magnetometer_z(
    const float & _arg)
  {
    this->magnetometer_z = _arg;
    return *this;
  }
  Type & set__roll_speed(
    const float & _arg)
  {
    this->roll_speed = _arg;
    return *this;
  }
  Type & set__pitch_speed(
    const float & _arg)
  {
    this->pitch_speed = _arg;
    return *this;
  }
  Type & set__heading_speed(
    const float & _arg)
  {
    this->heading_speed = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__q1(
    const float & _arg)
  {
    this->q1 = _arg;
    return *this;
  }
  Type & set__q2(
    const float & _arg)
  {
    this->q2 = _arg;
    return *this;
  }
  Type & set__q3(
    const float & _arg)
  {
    this->q3 = _arg;
    return *this;
  }
  Type & set__q4(
    const float & _arg)
  {
    this->q4 = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__pressure(
    const float & _arg)
  {
    this->pressure = _arg;
    return *this;
  }
  Type & set__pressure_temp(
    const float & _arg)
  {
    this->pressure_temp = _arg;
    return *this;
  }
  Type & set__timestamp(
    const int32_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::ImuData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::ImuData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::ImuData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::ImuData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__ImuData
    std::shared_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__ImuData
    std::shared_ptr<custom_interfaces::msg::ImuData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuData_ & other) const
  {
    if (this->gyroscope_x != other.gyroscope_x) {
      return false;
    }
    if (this->gyroscope_y != other.gyroscope_y) {
      return false;
    }
    if (this->gyroscope_z != other.gyroscope_z) {
      return false;
    }
    if (this->accelerometer_x != other.accelerometer_x) {
      return false;
    }
    if (this->accelerometer_y != other.accelerometer_y) {
      return false;
    }
    if (this->accelerometer_z != other.accelerometer_z) {
      return false;
    }
    if (this->magnetometer_x != other.magnetometer_x) {
      return false;
    }
    if (this->magnetometer_y != other.magnetometer_y) {
      return false;
    }
    if (this->magnetometer_z != other.magnetometer_z) {
      return false;
    }
    if (this->roll_speed != other.roll_speed) {
      return false;
    }
    if (this->pitch_speed != other.pitch_speed) {
      return false;
    }
    if (this->heading_speed != other.heading_speed) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->q1 != other.q1) {
      return false;
    }
    if (this->q2 != other.q2) {
      return false;
    }
    if (this->q3 != other.q3) {
      return false;
    }
    if (this->q4 != other.q4) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->pressure != other.pressure) {
      return false;
    }
    if (this->pressure_temp != other.pressure_temp) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuData_

// alias to use template instance with default allocator
using ImuData =
  custom_interfaces::msg::ImuData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
