// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuRaw.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_raw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgeImuRaw_compass_z
{
public:
  explicit Init_HedgeImuRaw_compass_z(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw compass_z(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_compass_z_type arg)
  {
    msg_.compass_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_compass_y
{
public:
  explicit Init_HedgeImuRaw_compass_y(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_compass_z compass_y(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_compass_y_type arg)
  {
    msg_.compass_y = std::move(arg);
    return Init_HedgeImuRaw_compass_z(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_compass_x
{
public:
  explicit Init_HedgeImuRaw_compass_x(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_compass_y compass_x(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_compass_x_type arg)
  {
    msg_.compass_x = std::move(arg);
    return Init_HedgeImuRaw_compass_y(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_gyro_z
{
public:
  explicit Init_HedgeImuRaw_gyro_z(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_compass_x gyro_z(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_gyro_z_type arg)
  {
    msg_.gyro_z = std::move(arg);
    return Init_HedgeImuRaw_compass_x(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_gyro_y
{
public:
  explicit Init_HedgeImuRaw_gyro_y(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_gyro_z gyro_y(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_gyro_y_type arg)
  {
    msg_.gyro_y = std::move(arg);
    return Init_HedgeImuRaw_gyro_z(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_gyro_x
{
public:
  explicit Init_HedgeImuRaw_gyro_x(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_gyro_y gyro_x(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_gyro_x_type arg)
  {
    msg_.gyro_x = std::move(arg);
    return Init_HedgeImuRaw_gyro_y(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_acc_z
{
public:
  explicit Init_HedgeImuRaw_acc_z(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_gyro_x acc_z(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_acc_z_type arg)
  {
    msg_.acc_z = std::move(arg);
    return Init_HedgeImuRaw_gyro_x(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_acc_y
{
public:
  explicit Init_HedgeImuRaw_acc_y(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_acc_z acc_y(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_acc_y_type arg)
  {
    msg_.acc_y = std::move(arg);
    return Init_HedgeImuRaw_acc_z(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_acc_x
{
public:
  explicit Init_HedgeImuRaw_acc_x(::marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
  : msg_(msg)
  {}
  Init_HedgeImuRaw_acc_y acc_x(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_acc_x_type arg)
  {
    msg_.acc_x = std::move(arg);
    return Init_HedgeImuRaw_acc_y(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

class Init_HedgeImuRaw_timestamp_ms
{
public:
  Init_HedgeImuRaw_timestamp_ms()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgeImuRaw_acc_x timestamp_ms(::marvelmind_ros2_msgs::msg::HedgeImuRaw::_timestamp_ms_type arg)
  {
    msg_.timestamp_ms = std::move(arg);
    return Init_HedgeImuRaw_acc_x(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuRaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgeImuRaw>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgeImuRaw_timestamp_ms();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__BUILDER_HPP_
