// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeTelemetry.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_telemetry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgeTelemetry_rssi_dbm
{
public:
  explicit Init_HedgeTelemetry_rssi_dbm(::marvelmind_ros2_msgs::msg::HedgeTelemetry & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgeTelemetry rssi_dbm(::marvelmind_ros2_msgs::msg::HedgeTelemetry::_rssi_dbm_type arg)
  {
    msg_.rssi_dbm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeTelemetry msg_;
};

class Init_HedgeTelemetry_battery_voltage
{
public:
  Init_HedgeTelemetry_battery_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgeTelemetry_rssi_dbm battery_voltage(::marvelmind_ros2_msgs::msg::HedgeTelemetry::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_HedgeTelemetry_rssi_dbm(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeTelemetry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgeTelemetry>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgeTelemetry_battery_voltage();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__BUILDER_HPP_
