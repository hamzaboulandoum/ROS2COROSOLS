// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/BeaconPositionAddressed.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_POSITION_ADDRESSED__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_POSITION_ADDRESSED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/beacon_position_addressed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_BeaconPositionAddressed_z_m
{
public:
  explicit Init_BeaconPositionAddressed_z_m(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::BeaconPositionAddressed z_m(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed::_z_m_type arg)
  {
    msg_.z_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconPositionAddressed msg_;
};

class Init_BeaconPositionAddressed_y_m
{
public:
  explicit Init_BeaconPositionAddressed_y_m(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed & msg)
  : msg_(msg)
  {}
  Init_BeaconPositionAddressed_z_m y_m(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_BeaconPositionAddressed_z_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconPositionAddressed msg_;
};

class Init_BeaconPositionAddressed_x_m
{
public:
  explicit Init_BeaconPositionAddressed_x_m(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed & msg)
  : msg_(msg)
  {}
  Init_BeaconPositionAddressed_y_m x_m(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_BeaconPositionAddressed_y_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconPositionAddressed msg_;
};

class Init_BeaconPositionAddressed_address
{
public:
  Init_BeaconPositionAddressed_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BeaconPositionAddressed_x_m address(::marvelmind_ros2_msgs::msg::BeaconPositionAddressed::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_BeaconPositionAddressed_x_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconPositionAddressed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::BeaconPositionAddressed>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_BeaconPositionAddressed_address();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_POSITION_ADDRESSED__BUILDER_HPP_
