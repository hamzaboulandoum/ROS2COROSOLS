// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeQuality.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_QUALITY__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_QUALITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_quality__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgeQuality_quality_percents
{
public:
  explicit Init_HedgeQuality_quality_percents(::marvelmind_ros2_msgs::msg::HedgeQuality & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgeQuality quality_percents(::marvelmind_ros2_msgs::msg::HedgeQuality::_quality_percents_type arg)
  {
    msg_.quality_percents = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeQuality msg_;
};

class Init_HedgeQuality_address
{
public:
  Init_HedgeQuality_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgeQuality_quality_percents address(::marvelmind_ros2_msgs::msg::HedgeQuality::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_HedgeQuality_quality_percents(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeQuality msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgeQuality>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgeQuality_address();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_QUALITY__BUILDER_HPP_
