// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from marvelmind_ros2_msgs:srv/MarvelmindAPI.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__TRAITS_HPP_
#define MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace marvelmind_ros2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MarvelmindAPI_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command_id
  {
    out << "command_id: ";
    rosidl_generator_traits::value_to_yaml(msg.command_id, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
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
  const MarvelmindAPI_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_id: ";
    rosidl_generator_traits::value_to_yaml(msg.command_id, out);
    out << "\n";
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
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

inline std::string to_yaml(const MarvelmindAPI_Request & msg, bool use_flow_style = false)
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

}  // namespace marvelmind_ros2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use marvelmind_ros2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const marvelmind_ros2_msgs::srv::MarvelmindAPI_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  marvelmind_ros2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use marvelmind_ros2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const marvelmind_ros2_msgs::srv::MarvelmindAPI_Request & msg)
{
  return marvelmind_ros2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>()
{
  return "marvelmind_ros2_msgs::srv::MarvelmindAPI_Request";
}

template<>
inline const char * name<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>()
{
  return "marvelmind_ros2_msgs/srv/MarvelmindAPI_Request";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace marvelmind_ros2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MarvelmindAPI_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const MarvelmindAPI_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
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

inline std::string to_yaml(const MarvelmindAPI_Response & msg, bool use_flow_style = false)
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

}  // namespace marvelmind_ros2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use marvelmind_ros2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const marvelmind_ros2_msgs::srv::MarvelmindAPI_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  marvelmind_ros2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use marvelmind_ros2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const marvelmind_ros2_msgs::srv::MarvelmindAPI_Response & msg)
{
  return marvelmind_ros2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>()
{
  return "marvelmind_ros2_msgs::srv::MarvelmindAPI_Response";
}

template<>
inline const char * name<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>()
{
  return "marvelmind_ros2_msgs/srv/MarvelmindAPI_Response";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<marvelmind_ros2_msgs::srv::MarvelmindAPI>()
{
  return "marvelmind_ros2_msgs::srv::MarvelmindAPI";
}

template<>
inline const char * name<marvelmind_ros2_msgs::srv::MarvelmindAPI>()
{
  return "marvelmind_ros2_msgs/srv/MarvelmindAPI";
}

template<>
struct has_fixed_size<marvelmind_ros2_msgs::srv::MarvelmindAPI>
  : std::integral_constant<
    bool,
    has_fixed_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>::value &&
    has_fixed_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>::value
  >
{
};

template<>
struct has_bounded_size<marvelmind_ros2_msgs::srv::MarvelmindAPI>
  : std::integral_constant<
    bool,
    has_bounded_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>::value &&
    has_bounded_size<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>::value
  >
{
};

template<>
struct is_service<marvelmind_ros2_msgs::srv::MarvelmindAPI>
  : std::true_type
{
};

template<>
struct is_service_request<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>
  : std::true_type
{
};

template<>
struct is_service_response<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__TRAITS_HPP_
