// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ImuData & msg,
  std::ostream & out)
{
  out << "{";
  // member: gyroscope_x
  {
    out << "gyroscope_x: ";
    rosidl_generator_traits::value_to_yaml(msg.gyroscope_x, out);
    out << ", ";
  }

  // member: gyroscope_y
  {
    out << "gyroscope_y: ";
    rosidl_generator_traits::value_to_yaml(msg.gyroscope_y, out);
    out << ", ";
  }

  // member: gyroscope_z
  {
    out << "gyroscope_z: ";
    rosidl_generator_traits::value_to_yaml(msg.gyroscope_z, out);
    out << ", ";
  }

  // member: accelerometer_x
  {
    out << "accelerometer_x: ";
    rosidl_generator_traits::value_to_yaml(msg.accelerometer_x, out);
    out << ", ";
  }

  // member: accelerometer_y
  {
    out << "accelerometer_y: ";
    rosidl_generator_traits::value_to_yaml(msg.accelerometer_y, out);
    out << ", ";
  }

  // member: accelerometer_z
  {
    out << "accelerometer_z: ";
    rosidl_generator_traits::value_to_yaml(msg.accelerometer_z, out);
    out << ", ";
  }

  // member: magnetometer_x
  {
    out << "magnetometer_x: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetometer_x, out);
    out << ", ";
  }

  // member: magnetometer_y
  {
    out << "magnetometer_y: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetometer_y, out);
    out << ", ";
  }

  // member: magnetometer_z
  {
    out << "magnetometer_z: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetometer_z, out);
    out << ", ";
  }

  // member: roll_speed
  {
    out << "roll_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_speed, out);
    out << ", ";
  }

  // member: pitch_speed
  {
    out << "pitch_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_speed, out);
    out << ", ";
  }

  // member: heading_speed
  {
    out << "heading_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_speed, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: q1
  {
    out << "q1: ";
    rosidl_generator_traits::value_to_yaml(msg.q1, out);
    out << ", ";
  }

  // member: q2
  {
    out << "q2: ";
    rosidl_generator_traits::value_to_yaml(msg.q2, out);
    out << ", ";
  }

  // member: q3
  {
    out << "q3: ";
    rosidl_generator_traits::value_to_yaml(msg.q3, out);
    out << ", ";
  }

  // member: q4
  {
    out << "q4: ";
    rosidl_generator_traits::value_to_yaml(msg.q4, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: pressure
  {
    out << "pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure, out);
    out << ", ";
  }

  // member: pressure_temp
  {
    out << "pressure_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_temp, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImuData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gyroscope_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyroscope_x: ";
    rosidl_generator_traits::value_to_yaml(msg.gyroscope_x, out);
    out << "\n";
  }

  // member: gyroscope_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyroscope_y: ";
    rosidl_generator_traits::value_to_yaml(msg.gyroscope_y, out);
    out << "\n";
  }

  // member: gyroscope_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyroscope_z: ";
    rosidl_generator_traits::value_to_yaml(msg.gyroscope_z, out);
    out << "\n";
  }

  // member: accelerometer_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accelerometer_x: ";
    rosidl_generator_traits::value_to_yaml(msg.accelerometer_x, out);
    out << "\n";
  }

  // member: accelerometer_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accelerometer_y: ";
    rosidl_generator_traits::value_to_yaml(msg.accelerometer_y, out);
    out << "\n";
  }

  // member: accelerometer_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accelerometer_z: ";
    rosidl_generator_traits::value_to_yaml(msg.accelerometer_z, out);
    out << "\n";
  }

  // member: magnetometer_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetometer_x: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetometer_x, out);
    out << "\n";
  }

  // member: magnetometer_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetometer_y: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetometer_y, out);
    out << "\n";
  }

  // member: magnetometer_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetometer_z: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetometer_z, out);
    out << "\n";
  }

  // member: roll_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_speed, out);
    out << "\n";
  }

  // member: pitch_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_speed, out);
    out << "\n";
  }

  // member: heading_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_speed, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: q1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q1: ";
    rosidl_generator_traits::value_to_yaml(msg.q1, out);
    out << "\n";
  }

  // member: q2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q2: ";
    rosidl_generator_traits::value_to_yaml(msg.q2, out);
    out << "\n";
  }

  // member: q3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q3: ";
    rosidl_generator_traits::value_to_yaml(msg.q3, out);
    out << "\n";
  }

  // member: q4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q4: ";
    rosidl_generator_traits::value_to_yaml(msg.q4, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: pressure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure, out);
    out << "\n";
  }

  // member: pressure_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressure_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_temp, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImuData & msg, bool use_flow_style = false)
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

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::msg::ImuData & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::ImuData & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::ImuData>()
{
  return "custom_interfaces::msg::ImuData";
}

template<>
inline const char * name<custom_interfaces::msg::ImuData>()
{
  return "custom_interfaces/msg/ImuData";
}

template<>
struct has_fixed_size<custom_interfaces::msg::ImuData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::msg::ImuData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::msg::ImuData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
