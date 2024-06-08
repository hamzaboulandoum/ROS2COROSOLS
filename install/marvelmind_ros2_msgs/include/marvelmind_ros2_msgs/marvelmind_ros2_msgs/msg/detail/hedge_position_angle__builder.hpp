// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgePositionAngle.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_position_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgePositionAngle_angle
{
public:
  explicit Init_HedgePositionAngle_angle(::marvelmind_ros2_msgs::msg::HedgePositionAngle & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle angle(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

class Init_HedgePositionAngle_flags
{
public:
  explicit Init_HedgePositionAngle_flags(::marvelmind_ros2_msgs::msg::HedgePositionAngle & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAngle_angle flags(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_HedgePositionAngle_angle(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

class Init_HedgePositionAngle_z_m
{
public:
  explicit Init_HedgePositionAngle_z_m(::marvelmind_ros2_msgs::msg::HedgePositionAngle & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAngle_flags z_m(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_z_m_type arg)
  {
    msg_.z_m = std::move(arg);
    return Init_HedgePositionAngle_flags(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

class Init_HedgePositionAngle_y_m
{
public:
  explicit Init_HedgePositionAngle_y_m(::marvelmind_ros2_msgs::msg::HedgePositionAngle & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAngle_z_m y_m(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_HedgePositionAngle_z_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

class Init_HedgePositionAngle_x_m
{
public:
  explicit Init_HedgePositionAngle_x_m(::marvelmind_ros2_msgs::msg::HedgePositionAngle & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAngle_y_m x_m(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_HedgePositionAngle_y_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

class Init_HedgePositionAngle_timestamp_ms
{
public:
  explicit Init_HedgePositionAngle_timestamp_ms(::marvelmind_ros2_msgs::msg::HedgePositionAngle & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAngle_x_m timestamp_ms(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_timestamp_ms_type arg)
  {
    msg_.timestamp_ms = std::move(arg);
    return Init_HedgePositionAngle_x_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

class Init_HedgePositionAngle_address
{
public:
  Init_HedgePositionAngle_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgePositionAngle_timestamp_ms address(::marvelmind_ros2_msgs::msg::HedgePositionAngle::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_HedgePositionAngle_timestamp_ms(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgePositionAngle>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgePositionAngle_address();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__BUILDER_HPP_
