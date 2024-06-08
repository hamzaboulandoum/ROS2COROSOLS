// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgePositionAddressed.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ADDRESSED__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ADDRESSED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_position_addressed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgePositionAddressed_flags
{
public:
  explicit Init_HedgePositionAddressed_flags(::marvelmind_ros2_msgs::msg::HedgePositionAddressed & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed flags(::marvelmind_ros2_msgs::msg::HedgePositionAddressed::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed msg_;
};

class Init_HedgePositionAddressed_z_m
{
public:
  explicit Init_HedgePositionAddressed_z_m(::marvelmind_ros2_msgs::msg::HedgePositionAddressed & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAddressed_flags z_m(::marvelmind_ros2_msgs::msg::HedgePositionAddressed::_z_m_type arg)
  {
    msg_.z_m = std::move(arg);
    return Init_HedgePositionAddressed_flags(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed msg_;
};

class Init_HedgePositionAddressed_y_m
{
public:
  explicit Init_HedgePositionAddressed_y_m(::marvelmind_ros2_msgs::msg::HedgePositionAddressed & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAddressed_z_m y_m(::marvelmind_ros2_msgs::msg::HedgePositionAddressed::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_HedgePositionAddressed_z_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed msg_;
};

class Init_HedgePositionAddressed_x_m
{
public:
  explicit Init_HedgePositionAddressed_x_m(::marvelmind_ros2_msgs::msg::HedgePositionAddressed & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAddressed_y_m x_m(::marvelmind_ros2_msgs::msg::HedgePositionAddressed::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_HedgePositionAddressed_y_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed msg_;
};

class Init_HedgePositionAddressed_timestamp_ms
{
public:
  explicit Init_HedgePositionAddressed_timestamp_ms(::marvelmind_ros2_msgs::msg::HedgePositionAddressed & msg)
  : msg_(msg)
  {}
  Init_HedgePositionAddressed_x_m timestamp_ms(::marvelmind_ros2_msgs::msg::HedgePositionAddressed::_timestamp_ms_type arg)
  {
    msg_.timestamp_ms = std::move(arg);
    return Init_HedgePositionAddressed_x_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed msg_;
};

class Init_HedgePositionAddressed_address
{
public:
  Init_HedgePositionAddressed_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgePositionAddressed_timestamp_ms address(::marvelmind_ros2_msgs::msg::HedgePositionAddressed::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_HedgePositionAddressed_timestamp_ms(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgePositionAddressed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgePositionAddressed>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgePositionAddressed_address();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ADDRESSED__BUILDER_HPP_
