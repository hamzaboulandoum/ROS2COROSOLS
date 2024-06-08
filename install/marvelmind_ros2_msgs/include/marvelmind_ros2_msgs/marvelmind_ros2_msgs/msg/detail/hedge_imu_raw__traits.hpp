// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuRaw.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__TRAITS_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_raw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace marvelmind_ros2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HedgeImuRaw & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp_ms
  {
    out << "timestamp_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_ms, out);
    out << ", ";
  }

  // member: acc_x
  {
    out << "acc_x: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_x, out);
    out << ", ";
  }

  // member: acc_y
  {
    out << "acc_y: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_y, out);
    out << ", ";
  }

  // member: acc_z
  {
    out << "acc_z: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_z, out);
    out << ", ";
  }

  // member: gyro_x
  {
    out << "gyro_x: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_x, out);
    out << ", ";
  }

  // member: gyro_y
  {
    out << "gyro_y: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_y, out);
    out << ", ";
  }

  // member: gyro_z
  {
    out << "gyro_z: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_z, out);
    out << ", ";
  }

  // member: compass_x
  {
    out << "compass_x: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_x, out);
    out << ", ";
  }

  // member: compass_y
  {
    out << "compass_y: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_y, out);
    out << ", ";
  }

  // member: compass_z
  {
    out << "compass_z: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HedgeImuRaw & msg,
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

  // member: acc_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_x: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_x, out);
    out << "\n";
  }

  // member: acc_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_y: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_y, out);
    out << "\n";
  }

  // member: acc_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_z: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_z, out);
    out << "\n";
  }

  // member: gyro_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_x: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_x, out);
    out << "\n";
  }

  // member: gyro_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_y: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_y, out);
    out << "\n";
  }

  // member: gyro_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_z: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_z, out);
    out << "\n";
  }

  // member: compass_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compass_x: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_x, out);
    out << "\n";
  }

  // member: compass_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compass_y: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_y, out);
    out << "\n";
  }

  // member: compass_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compass_z: ";
    rosidl_generator_traits::value_to_yaml(msg.compass_z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HedgeImuRaw & msg, bool use_flow_style = false)
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
  const marvelmind_ros2_msgs::msg::HedgeImuRaw & msg,
  std::ostream & out, size_t indentation = 0)
{
  marvelmind_ros2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use marvelmind_ros2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const marvelmind_ros2_msgs::msg::HedgeImuRaw & msg)
{
  return marvelmind_ros2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<marvelmind_ros2_msgs::msg::HedgeImuRaw>()
{
  return "marvelmind_ros2_msgs::msg::HedgeImuRaw";
}

template<>
inline const char * name<marvelmind_ros2_msgs::msg::HedgeImuRaw>()
{
  return "marvelmind_ros2_msgs/msg/HedgeImuRaw";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::msg::HedgeImuRaw>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::msg::HedgeImuRaw>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<marvelmind_ros2_msgs::msg::HedgeImuRaw>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__TRAITS_HPP_
