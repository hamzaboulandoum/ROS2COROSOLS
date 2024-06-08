// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/BeaconDistance.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/beacon_distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_BeaconDistance_distance_m
{
public:
  explicit Init_BeaconDistance_distance_m(::marvelmind_ros2_msgs::msg::BeaconDistance & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::BeaconDistance distance_m(::marvelmind_ros2_msgs::msg::BeaconDistance::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconDistance msg_;
};

class Init_BeaconDistance_address_beacon
{
public:
  explicit Init_BeaconDistance_address_beacon(::marvelmind_ros2_msgs::msg::BeaconDistance & msg)
  : msg_(msg)
  {}
  Init_BeaconDistance_distance_m address_beacon(::marvelmind_ros2_msgs::msg::BeaconDistance::_address_beacon_type arg)
  {
    msg_.address_beacon = std::move(arg);
    return Init_BeaconDistance_distance_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconDistance msg_;
};

class Init_BeaconDistance_address_hedge
{
public:
  Init_BeaconDistance_address_hedge()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BeaconDistance_address_beacon address_hedge(::marvelmind_ros2_msgs::msg::BeaconDistance::_address_hedge_type arg)
  {
    msg_.address_hedge = std::move(arg);
    return Init_BeaconDistance_address_beacon(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::BeaconDistance msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::BeaconDistance>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_BeaconDistance_address_hedge();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__BUILDER_HPP_
