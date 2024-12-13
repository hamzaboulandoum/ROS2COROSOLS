// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_ImuData_timestamp
{
public:
  explicit Init_ImuData_timestamp(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::ImuData timestamp(::custom_interfaces::msg::ImuData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_pressure_temp
{
public:
  explicit Init_ImuData_pressure_temp(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_timestamp pressure_temp(::custom_interfaces::msg::ImuData::_pressure_temp_type arg)
  {
    msg_.pressure_temp = std::move(arg);
    return Init_ImuData_timestamp(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_pressure
{
public:
  explicit Init_ImuData_pressure(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_pressure_temp pressure(::custom_interfaces::msg::ImuData::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_ImuData_pressure_temp(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_temperature
{
public:
  explicit Init_ImuData_temperature(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_pressure temperature(::custom_interfaces::msg::ImuData::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_ImuData_pressure(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_q4
{
public:
  explicit Init_ImuData_q4(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_temperature q4(::custom_interfaces::msg::ImuData::_q4_type arg)
  {
    msg_.q4 = std::move(arg);
    return Init_ImuData_temperature(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_q3
{
public:
  explicit Init_ImuData_q3(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_q4 q3(::custom_interfaces::msg::ImuData::_q3_type arg)
  {
    msg_.q3 = std::move(arg);
    return Init_ImuData_q4(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_q2
{
public:
  explicit Init_ImuData_q2(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_q3 q2(::custom_interfaces::msg::ImuData::_q2_type arg)
  {
    msg_.q2 = std::move(arg);
    return Init_ImuData_q3(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_q1
{
public:
  explicit Init_ImuData_q1(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_q2 q1(::custom_interfaces::msg::ImuData::_q1_type arg)
  {
    msg_.q1 = std::move(arg);
    return Init_ImuData_q2(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_heading
{
public:
  explicit Init_ImuData_heading(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_q1 heading(::custom_interfaces::msg::ImuData::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_ImuData_q1(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_pitch
{
public:
  explicit Init_ImuData_pitch(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_heading pitch(::custom_interfaces::msg::ImuData::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ImuData_heading(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_roll
{
public:
  explicit Init_ImuData_roll(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_pitch roll(::custom_interfaces::msg::ImuData::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_ImuData_pitch(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_heading_speed
{
public:
  explicit Init_ImuData_heading_speed(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_roll heading_speed(::custom_interfaces::msg::ImuData::_heading_speed_type arg)
  {
    msg_.heading_speed = std::move(arg);
    return Init_ImuData_roll(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_pitch_speed
{
public:
  explicit Init_ImuData_pitch_speed(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_heading_speed pitch_speed(::custom_interfaces::msg::ImuData::_pitch_speed_type arg)
  {
    msg_.pitch_speed = std::move(arg);
    return Init_ImuData_heading_speed(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_roll_speed
{
public:
  explicit Init_ImuData_roll_speed(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_pitch_speed roll_speed(::custom_interfaces::msg::ImuData::_roll_speed_type arg)
  {
    msg_.roll_speed = std::move(arg);
    return Init_ImuData_pitch_speed(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_magnetometer_z
{
public:
  explicit Init_ImuData_magnetometer_z(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_roll_speed magnetometer_z(::custom_interfaces::msg::ImuData::_magnetometer_z_type arg)
  {
    msg_.magnetometer_z = std::move(arg);
    return Init_ImuData_roll_speed(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_magnetometer_y
{
public:
  explicit Init_ImuData_magnetometer_y(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_magnetometer_z magnetometer_y(::custom_interfaces::msg::ImuData::_magnetometer_y_type arg)
  {
    msg_.magnetometer_y = std::move(arg);
    return Init_ImuData_magnetometer_z(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_magnetometer_x
{
public:
  explicit Init_ImuData_magnetometer_x(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_magnetometer_y magnetometer_x(::custom_interfaces::msg::ImuData::_magnetometer_x_type arg)
  {
    msg_.magnetometer_x = std::move(arg);
    return Init_ImuData_magnetometer_y(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_accelerometer_z
{
public:
  explicit Init_ImuData_accelerometer_z(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_magnetometer_x accelerometer_z(::custom_interfaces::msg::ImuData::_accelerometer_z_type arg)
  {
    msg_.accelerometer_z = std::move(arg);
    return Init_ImuData_magnetometer_x(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_accelerometer_y
{
public:
  explicit Init_ImuData_accelerometer_y(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_accelerometer_z accelerometer_y(::custom_interfaces::msg::ImuData::_accelerometer_y_type arg)
  {
    msg_.accelerometer_y = std::move(arg);
    return Init_ImuData_accelerometer_z(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_accelerometer_x
{
public:
  explicit Init_ImuData_accelerometer_x(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_accelerometer_y accelerometer_x(::custom_interfaces::msg::ImuData::_accelerometer_x_type arg)
  {
    msg_.accelerometer_x = std::move(arg);
    return Init_ImuData_accelerometer_y(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_gyroscope_z
{
public:
  explicit Init_ImuData_gyroscope_z(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_accelerometer_x gyroscope_z(::custom_interfaces::msg::ImuData::_gyroscope_z_type arg)
  {
    msg_.gyroscope_z = std::move(arg);
    return Init_ImuData_accelerometer_x(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_gyroscope_y
{
public:
  explicit Init_ImuData_gyroscope_y(::custom_interfaces::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_gyroscope_z gyroscope_y(::custom_interfaces::msg::ImuData::_gyroscope_y_type arg)
  {
    msg_.gyroscope_y = std::move(arg);
    return Init_ImuData_gyroscope_z(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

class Init_ImuData_gyroscope_x
{
public:
  Init_ImuData_gyroscope_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuData_gyroscope_y gyroscope_x(::custom_interfaces::msg::ImuData::_gyroscope_x_type arg)
  {
    msg_.gyroscope_x = std::move(arg);
    return Init_ImuData_gyroscope_y(msg_);
  }

private:
  ::custom_interfaces::msg::ImuData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::ImuData>()
{
  return custom_interfaces::msg::builder::Init_ImuData_gyroscope_x();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
