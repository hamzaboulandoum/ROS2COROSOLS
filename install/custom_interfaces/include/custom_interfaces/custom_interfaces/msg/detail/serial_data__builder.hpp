// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/serial_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_SerialData_stepper_y
{
public:
  explicit Init_SerialData_stepper_y(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::SerialData stepper_y(::custom_interfaces::msg::SerialData::_stepper_y_type arg)
  {
    msg_.stepper_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_stepper_x
{
public:
  explicit Init_SerialData_stepper_x(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_stepper_y stepper_x(::custom_interfaces::msg::SerialData::_stepper_x_type arg)
  {
    msg_.stepper_x = std::move(arg);
    return Init_SerialData_stepper_y(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_power_voltage
{
public:
  explicit Init_SerialData_power_voltage(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_stepper_x power_voltage(::custom_interfaces::msg::SerialData::_power_voltage_type arg)
  {
    msg_.power_voltage = std::move(arg);
    return Init_SerialData_stepper_x(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_roll
{
public:
  explicit Init_SerialData_roll(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_power_voltage roll(::custom_interfaces::msg::SerialData::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_SerialData_power_voltage(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_pitch
{
public:
  explicit Init_SerialData_pitch(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_roll pitch(::custom_interfaces::msg::SerialData::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SerialData_roll(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_heading
{
public:
  explicit Init_SerialData_heading(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_pitch heading(::custom_interfaces::msg::SerialData::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_SerialData_pitch(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_z_accel
{
public:
  explicit Init_SerialData_z_accel(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_heading z_accel(::custom_interfaces::msg::SerialData::_z_accel_type arg)
  {
    msg_.z_accel = std::move(arg);
    return Init_SerialData_heading(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_y_accel
{
public:
  explicit Init_SerialData_y_accel(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_z_accel y_accel(::custom_interfaces::msg::SerialData::_y_accel_type arg)
  {
    msg_.y_accel = std::move(arg);
    return Init_SerialData_z_accel(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_x_accel
{
public:
  explicit Init_SerialData_x_accel(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_y_accel x_accel(::custom_interfaces::msg::SerialData::_x_accel_type arg)
  {
    msg_.x_accel = std::move(arg);
    return Init_SerialData_y_accel(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_z_speed
{
public:
  explicit Init_SerialData_z_speed(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_x_accel z_speed(::custom_interfaces::msg::SerialData::_z_speed_type arg)
  {
    msg_.z_speed = std::move(arg);
    return Init_SerialData_x_accel(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_y_speed
{
public:
  explicit Init_SerialData_y_speed(::custom_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_z_speed y_speed(::custom_interfaces::msg::SerialData::_y_speed_type arg)
  {
    msg_.y_speed = std::move(arg);
    return Init_SerialData_z_speed(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

class Init_SerialData_x_speed
{
public:
  Init_SerialData_x_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SerialData_y_speed x_speed(::custom_interfaces::msg::SerialData::_x_speed_type arg)
  {
    msg_.x_speed = std::move(arg);
    return Init_SerialData_y_speed(msg_);
  }

private:
  ::custom_interfaces::msg::SerialData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::SerialData>()
{
  return custom_interfaces::msg::builder::Init_SerialData_x_speed();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_
