// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__TRAITS_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "marvelmind_ros2_msgs/msg/detail/marvelmind_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace marvelmind_ros2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MarvelmindWaypoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_items
  {
    out << "total_items: ";
    rosidl_generator_traits::value_to_yaml(msg.total_items, out);
    out << ", ";
  }

  // member: item_index
  {
    out << "item_index: ";
    rosidl_generator_traits::value_to_yaml(msg.item_index, out);
    out << ", ";
  }

  // member: movement_type
  {
    out << "movement_type: ";
    rosidl_generator_traits::value_to_yaml(msg.movement_type, out);
    out << ", ";
  }

  // member: param1
  {
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << ", ";
  }

  // member: param2
  {
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << ", ";
  }

  // member: param3
  {
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MarvelmindWaypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_items
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_items: ";
    rosidl_generator_traits::value_to_yaml(msg.total_items, out);
    out << "\n";
  }

  // member: item_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "item_index: ";
    rosidl_generator_traits::value_to_yaml(msg.item_index, out);
    out << "\n";
  }

  // member: movement_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "movement_type: ";
    rosidl_generator_traits::value_to_yaml(msg.movement_type, out);
    out << "\n";
  }

  // member: param1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << "\n";
  }

  // member: param2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << "\n";
  }

  // member: param3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MarvelmindWaypoint & msg, bool use_flow_style = false)
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
  const marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  marvelmind_ros2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use marvelmind_ros2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const marvelmind_ros2_msgs::msg::MarvelmindWaypoint & msg)
{
  return marvelmind_ros2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>()
{
  return "marvelmind_ros2_msgs::msg::MarvelmindWaypoint";
}

template<>
inline const char * name<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>()
{
  return "marvelmind_ros2_msgs/msg/MarvelmindWaypoint";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__TRAITS_HPP_
