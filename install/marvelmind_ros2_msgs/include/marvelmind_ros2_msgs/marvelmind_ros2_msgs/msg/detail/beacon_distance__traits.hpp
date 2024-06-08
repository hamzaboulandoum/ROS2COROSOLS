// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from marvelmind_ros2_msgs:msg/BeaconDistance.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__TRAITS_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "marvelmind_ros2_msgs/msg/detail/beacon_distance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace marvelmind_ros2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BeaconDistance & msg,
  std::ostream & out)
{
  out << "{";
  // member: address_hedge
  {
    out << "address_hedge: ";
    rosidl_generator_traits::value_to_yaml(msg.address_hedge, out);
    out << ", ";
  }

  // member: address_beacon
  {
    out << "address_beacon: ";
    rosidl_generator_traits::value_to_yaml(msg.address_beacon, out);
    out << ", ";
  }

  // member: distance_m
  {
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BeaconDistance & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: address_hedge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "address_hedge: ";
    rosidl_generator_traits::value_to_yaml(msg.address_hedge, out);
    out << "\n";
  }

  // member: address_beacon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "address_beacon: ";
    rosidl_generator_traits::value_to_yaml(msg.address_beacon, out);
    out << "\n";
  }

  // member: distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BeaconDistance & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace marvelmind_ros2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use marvelmind_ros2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const marvelmind_ros2_msgs::msg::BeaconDistance & msg,
  std::ostream & out, size_t indentation = 0)
{
  marvelmind_ros2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use marvelmind_ros2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const marvelmind_ros2_msgs::msg::BeaconDistance & msg)
{
  return marvelmind_ros2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<marvelmind_ros2_msgs::msg::BeaconDistance>()
{
  return "marvelmind_ros2_msgs::msg::BeaconDistance";
}

template<>
inline const char * name<marvelmind_ros2_msgs::msg::BeaconDistance>()
{
  return "marvelmind_ros2_msgs/msg/BeaconDistance";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::msg::BeaconDistance>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::msg::BeaconDistance>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<marvelmind_ros2_msgs::msg::BeaconDistance>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__BEACON_DISTANCE__TRAITS_HPP_
