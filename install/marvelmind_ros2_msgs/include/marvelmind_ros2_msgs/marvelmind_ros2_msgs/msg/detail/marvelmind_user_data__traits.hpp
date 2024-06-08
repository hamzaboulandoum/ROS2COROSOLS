// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindUserData.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__TRAITS_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "marvelmind_ros2_msgs/msg/detail/marvelmind_user_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace marvelmind_ros2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MarvelmindUserData & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp_ms
  {
    out << "timestamp_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_ms, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MarvelmindUserData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_ms, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MarvelmindUserData & msg, bool use_flow_style = false)
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
  const marvelmind_ros2_msgs::msg::MarvelmindUserData & msg,
  std::ostream & out, size_t indentation = 0)
{
  marvelmind_ros2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use marvelmind_ros2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const marvelmind_ros2_msgs::msg::MarvelmindUserData & msg)
{
  return marvelmind_ros2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<marvelmind_ros2_msgs::msg::MarvelmindUserData>()
{
  return "marvelmind_ros2_msgs::msg::MarvelmindUserData";
}

template<>
inline const char * name<marvelmind_ros2_msgs::msg::MarvelmindUserData>()
{
  return "marvelmind_ros2_msgs/msg/MarvelmindUserData";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::msg::MarvelmindUserData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::msg::MarvelmindUserData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<marvelmind_ros2_msgs::msg::MarvelmindUserData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__TRAITS_HPP_
