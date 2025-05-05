// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:srv/Commands.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__TRAITS_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/srv/detail/commands__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Commands_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: vr
  {
    out << "vr: ";
    rosidl_generator_traits::value_to_yaml(msg.vr, out);
    out << ", ";
  }

  // member: stepperx
  {
    out << "stepperx: ";
    rosidl_generator_traits::value_to_yaml(msg.stepperx, out);
    out << ", ";
  }

  // member: steppery
  {
    out << "steppery: ";
    rosidl_generator_traits::value_to_yaml(msg.steppery, out);
    out << ", ";
  }

  // member: airbrush
  {
    out << "airbrush: ";
    rosidl_generator_traits::value_to_yaml(msg.airbrush, out);
    out << ", ";
  }

  // member: ab_servo
  {
    out << "ab_servo: ";
    rosidl_generator_traits::value_to_yaml(msg.ab_servo, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Commands_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: vr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vr: ";
    rosidl_generator_traits::value_to_yaml(msg.vr, out);
    out << "\n";
  }

  // member: stepperx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stepperx: ";
    rosidl_generator_traits::value_to_yaml(msg.stepperx, out);
    out << "\n";
  }

  // member: steppery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steppery: ";
    rosidl_generator_traits::value_to_yaml(msg.steppery, out);
    out << "\n";
  }

  // member: airbrush
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "airbrush: ";
    rosidl_generator_traits::value_to_yaml(msg.airbrush, out);
    out << "\n";
  }

  // member: ab_servo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ab_servo: ";
    rosidl_generator_traits::value_to_yaml(msg.ab_servo, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Commands_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::Commands_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::Commands_Request & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::Commands_Request>()
{
  return "custom_interfaces::srv::Commands_Request";
}

template<>
inline const char * name<custom_interfaces::srv::Commands_Request>()
{
  return "custom_interfaces/srv/Commands_Request";
}

template<>
struct has_fixed_size<custom_interfaces::srv::Commands_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::srv::Commands_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::srv::Commands_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Commands_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: outcome
  {
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Commands_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: outcome
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Commands_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::Commands_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::Commands_Response & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::Commands_Response>()
{
  return "custom_interfaces::srv::Commands_Response";
}

template<>
inline const char * name<custom_interfaces::srv::Commands_Response>()
{
  return "custom_interfaces/srv/Commands_Response";
}

template<>
struct has_fixed_size<custom_interfaces::srv::Commands_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interfaces::srv::Commands_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interfaces::srv::Commands_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interfaces::srv::Commands>()
{
  return "custom_interfaces::srv::Commands";
}

template<>
inline const char * name<custom_interfaces::srv::Commands>()
{
  return "custom_interfaces/srv/Commands";
}

template<>
struct has_fixed_size<custom_interfaces::srv::Commands>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interfaces::srv::Commands_Request>::value &&
    has_fixed_size<custom_interfaces::srv::Commands_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interfaces::srv::Commands>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interfaces::srv::Commands_Request>::value &&
    has_bounded_size<custom_interfaces::srv::Commands_Response>::value
  >
{
};

template<>
struct is_service<custom_interfaces::srv::Commands>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interfaces::srv::Commands_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interfaces::srv::Commands_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__TRAITS_HPP_
