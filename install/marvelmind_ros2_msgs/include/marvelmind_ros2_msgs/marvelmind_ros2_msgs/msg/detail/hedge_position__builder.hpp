// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgePosition.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgePosition_flags
{
public:
  explicit Init_HedgePosition_flags(::marvelmind_ros2_msgs::msg::HedgePosition & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgePosition flags(::marvelmind_ros2_msgs::msg::HedgePosition::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePosition msg_;
};

class Init_HedgePosition_z_m
{
public:
  explicit Init_HedgePosition_z_m(::marvelmind_ros2_msgs::msg::HedgePosition & msg)
  : msg_(msg)
  {}
  Init_HedgePosition_flags z_m(::marvelmind_ros2_msgs::msg::HedgePosition::_z_m_type arg)
  {
    msg_.z_m = std::move(arg);
    return Init_HedgePosition_flags(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePosition msg_;
};

class Init_HedgePosition_y_m
{
public:
  explicit Init_HedgePosition_y_m(::marvelmind_ros2_msgs::msg::HedgePosition & msg)
  : msg_(msg)
  {}
  Init_HedgePosition_z_m y_m(::marvelmind_ros2_msgs::msg::HedgePosition::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_HedgePosition_z_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePosition msg_;
};

class Init_HedgePosition_x_m
{
public:
  explicit Init_HedgePosition_x_m(::marvelmind_ros2_msgs::msg::HedgePosition & msg)
  : msg_(msg)
  {}
  Init_HedgePosition_y_m x_m(::marvelmind_ros2_msgs::msg::HedgePosition::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_HedgePosition_y_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePosition msg_;
};

class Init_HedgePosition_timestamp_ms
{
public:
  Init_HedgePosition_timestamp_ms()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgePosition_x_m timestamp_ms(::marvelmind_ros2_msgs::msg::HedgePosition::_timestamp_ms_type arg)
  {
    msg_.timestamp_ms = std::move(arg);
    return Init_HedgePosition_x_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgePosition>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgePosition_timestamp_ms();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION__BUILDER_HPP_
