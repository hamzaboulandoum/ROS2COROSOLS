// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/serial_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SerialData & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_speed
  {
    out << "x_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.x_speed, out);
    out << ", ";
  }

  // member: y_speed
  {
    out << "y_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.y_speed, out);
    out << ", ";
  }

  // member: z_speed
  {
    out << "z_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.z_speed, out);
    out << ", ";
  }

  // member: x_accel
  {
    out << "x_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.x_accel, out);
    out << ", ";
  }

  // member: y_accel
  {
    out << "y_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.y_accel, out);
    out << ", ";
  }

  // member: z_accel
  {
    out << "z_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.z_accel, out);
    out << ", ";
  }

  // member: x_gyro
  {
    out << "x_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.x_gyro, out);
    out << ", ";
  }

  // member: y_gyro
  {
    out << "y_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.y_gyro, out);
    out << ", ";
  }

  // member: z_gyro
  {
    out << "z_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.z_gyro, out);
    out << ", ";
  }

  // member: power_voltage
  {
    out << "power_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.power_voltage, out);
    out << ", ";
  }

  // member: stepper_x
  {
    out << "stepper_x: ";
    rosidl_generator_traits::value_to_yaml(msg.stepper_x, out);
    out << ", ";
  }

  // member: stepper_y
  {
    out << "stepper_y: ";
    rosidl_generator_traits::value_to_yaml(msg.stepper_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SerialData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.x_speed, out);
    out << "\n";
  }

  // member: y_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.y_speed, out);
    out << "\n";
  }

  // member: z_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.z_speed, out);
    out << "\n";
  }

  // member: x_accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.x_accel, out);
    out << "\n";
  }

  // member: y_accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.y_accel, out);
    out << "\n";
  }

  // member: z_accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.z_accel, out);
    out << "\n";
  }

  // member: x_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.x_gyro, out);
    out << "\n";
  }

  // member: y_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.y_gyro, out);
    out << "\n";
  }

  // member: z_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.z_gyro, out);
    out << "\n";
  }

  // member: power_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.power_voltage, out);
    out << "\n";
  }

  // member: stepper_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stepper_x: ";
    rosidl_generator_traits::value_to_yaml(msg.stepper_x, out);
    out << "\n";
  }

  // member: stepper_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stepper_y: ";
    rosidl_generator_traits::value_to_yaml(msg.stepper_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SerialData & msg, bool use_flow_style = false)
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
  const custom_interfaces::msg::SerialData & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::SerialData & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::SerialData>()
{
  return "custom_interfaces::msg::SerialData";
}

template<>
inline const char * name<custom_interfaces::msg::SerialData>()
{
  return "custom_interfaces/msg/SerialData";
}

template<>
struct has_fixed_size<custom_interfaces::msg::SerialData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::msg::SerialData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::msg::SerialData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_
