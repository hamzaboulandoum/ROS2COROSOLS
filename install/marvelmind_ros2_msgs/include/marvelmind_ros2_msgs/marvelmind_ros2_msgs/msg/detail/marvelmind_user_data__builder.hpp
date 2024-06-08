// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindUserData.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/marvelmind_user_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_MarvelmindUserData_data
{
public:
  explicit Init_MarvelmindUserData_data(::marvelmind_ros2_msgs::msg::MarvelmindUserData & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::MarvelmindUserData data(::marvelmind_ros2_msgs::msg::MarvelmindUserData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindUserData msg_;
};

class Init_MarvelmindUserData_timestamp_ms
{
public:
  Init_MarvelmindUserData_timestamp_ms()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarvelmindUserData_data timestamp_ms(::marvelmind_ros2_msgs::msg::MarvelmindUserData::_timestamp_ms_type arg)
  {
    msg_.timestamp_ms = std::move(arg);
    return Init_MarvelmindUserData_data(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindUserData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::MarvelmindUserData>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_MarvelmindUserData_timestamp_ms();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__BUILDER_HPP_
