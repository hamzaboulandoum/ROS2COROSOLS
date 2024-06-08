// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/marvelmind_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_MarvelmindWaypoint_param3
{
public:
  explicit Init_MarvelmindWaypoint_param3(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint param3(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint::_param3_type arg)
  {
    msg_.param3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint msg_;
};

class Init_MarvelmindWaypoint_param2
{
public:
  explicit Init_MarvelmindWaypoint_param2(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg)
  : msg_(msg)
  {}
  Init_MarvelmindWaypoint_param3 param2(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint::_param2_type arg)
  {
    msg_.param2 = std::move(arg);
    return Init_MarvelmindWaypoint_param3(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint msg_;
};

class Init_MarvelmindWaypoint_param1
{
public:
  explicit Init_MarvelmindWaypoint_param1(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg)
  : msg_(msg)
  {}
  Init_MarvelmindWaypoint_param2 param1(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint::_param1_type arg)
  {
    msg_.param1 = std::move(arg);
    return Init_MarvelmindWaypoint_param2(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint msg_;
};

class Init_MarvelmindWaypoint_movement_type
{
public:
  explicit Init_MarvelmindWaypoint_movement_type(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg)
  : msg_(msg)
  {}
  Init_MarvelmindWaypoint_param1 movement_type(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint::_movement_type_type arg)
  {
    msg_.movement_type = std::move(arg);
    return Init_MarvelmindWaypoint_param1(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint msg_;
};

class Init_MarvelmindWaypoint_item_index
{
public:
  explicit Init_MarvelmindWaypoint_item_index(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg)
  : msg_(msg)
  {}
  Init_MarvelmindWaypoint_movement_type item_index(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint::_item_index_type arg)
  {
    msg_.item_index = std::move(arg);
    return Init_MarvelmindWaypoint_movement_type(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint msg_;
};

class Init_MarvelmindWaypoint_total_items
{
public:
  Init_MarvelmindWaypoint_total_items()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarvelmindWaypoint_item_index total_items(::marvelmind_ros2_msgs::msg::MarvelmindWaypoint::_total_items_type arg)
  {
    msg_.total_items = std::move(arg);
    return Init_MarvelmindWaypoint_item_index(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::MarvelmindWaypoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::MarvelmindWaypoint>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_MarvelmindWaypoint_total_items();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__BUILDER_HPP_
