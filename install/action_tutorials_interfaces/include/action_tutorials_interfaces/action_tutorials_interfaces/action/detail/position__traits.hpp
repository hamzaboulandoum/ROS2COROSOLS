// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from action_tutorials_interfaces:action/Position.idl
// generated code does not contain a copyright notice

#ifndef ACTION_TUTORIALS_INTERFACES__ACTION__DETAIL__POSITION__TRAITS_HPP_
#define ACTION_TUTORIALS_INTERFACES__ACTION__DETAIL__POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "action_tutorials_interfaces/action/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_Goal & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_Goal>()
{
  return "action_tutorials_interfaces::action::Position_Goal";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_Goal>()
{
  return "action_tutorials_interfaces/action/Position_Goal";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: precision
  {
    if (msg.precision.size() == 0) {
      out << "precision: []";
    } else {
      out << "precision: [";
      size_t pending_items = msg.precision.size();
      for (auto item : msg.precision) {
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
  const Position_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: precision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.precision.size() == 0) {
      out << "precision: []\n";
    } else {
      out << "precision:\n";
      for (auto item : msg.precision) {
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

inline std::string to_yaml(const Position_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_Result & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_Result>()
{
  return "action_tutorials_interfaces::action::Position_Result";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_Result>()
{
  return "action_tutorials_interfaces/action/Position_Result";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: precision_log
  {
    if (msg.precision_log.size() == 0) {
      out << "precision_log: []";
    } else {
      out << "precision_log: [";
      size_t pending_items = msg.precision_log.size();
      for (auto item : msg.precision_log) {
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
  const Position_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: precision_log
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.precision_log.size() == 0) {
      out << "precision_log: []\n";
    } else {
      out << "precision_log:\n";
      for (auto item : msg.precision_log) {
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

inline std::string to_yaml(const Position_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_Feedback & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_Feedback>()
{
  return "action_tutorials_interfaces::action::Position_Feedback";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_Feedback>()
{
  return "action_tutorials_interfaces/action/Position_Feedback";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "action_tutorials_interfaces/action/detail/position__traits.hpp"

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_SendGoal_Request & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_SendGoal_Request>()
{
  return "action_tutorials_interfaces::action::Position_SendGoal_Request";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_SendGoal_Request>()
{
  return "action_tutorials_interfaces/action/Position_SendGoal_Request";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<action_tutorials_interfaces::action::Position_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<action_tutorials_interfaces::action::Position_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_SendGoal_Response & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_SendGoal_Response>()
{
  return "action_tutorials_interfaces::action::Position_SendGoal_Response";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_SendGoal_Response>()
{
  return "action_tutorials_interfaces/action/Position_SendGoal_Response";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_SendGoal>()
{
  return "action_tutorials_interfaces::action::Position_SendGoal";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_SendGoal>()
{
  return "action_tutorials_interfaces/action/Position_SendGoal";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<action_tutorials_interfaces::action::Position_SendGoal_Request>::value &&
    has_fixed_size<action_tutorials_interfaces::action::Position_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<action_tutorials_interfaces::action::Position_SendGoal_Request>::value &&
    has_bounded_size<action_tutorials_interfaces::action::Position_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<action_tutorials_interfaces::action::Position_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<action_tutorials_interfaces::action::Position_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_tutorials_interfaces::action::Position_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_GetResult_Request & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_GetResult_Request>()
{
  return "action_tutorials_interfaces::action::Position_GetResult_Request";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_GetResult_Request>()
{
  return "action_tutorials_interfaces/action/Position_GetResult_Request";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "action_tutorials_interfaces/action/detail/position__traits.hpp"

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_GetResult_Response & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_GetResult_Response>()
{
  return "action_tutorials_interfaces::action::Position_GetResult_Response";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_GetResult_Response>()
{
  return "action_tutorials_interfaces/action/Position_GetResult_Response";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<action_tutorials_interfaces::action::Position_Result>::value> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<action_tutorials_interfaces::action::Position_Result>::value> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_GetResult>()
{
  return "action_tutorials_interfaces::action::Position_GetResult";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_GetResult>()
{
  return "action_tutorials_interfaces/action/Position_GetResult";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<action_tutorials_interfaces::action::Position_GetResult_Request>::value &&
    has_fixed_size<action_tutorials_interfaces::action::Position_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<action_tutorials_interfaces::action::Position_GetResult_Request>::value &&
    has_bounded_size<action_tutorials_interfaces::action::Position_GetResult_Response>::value
  >
{
};

template<>
struct is_service<action_tutorials_interfaces::action::Position_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<action_tutorials_interfaces::action::Position_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_tutorials_interfaces::action::Position_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "action_tutorials_interfaces/action/detail/position__traits.hpp"

namespace action_tutorials_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Position_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_tutorials_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use action_tutorials_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_tutorials_interfaces::action::Position_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_tutorials_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_tutorials_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const action_tutorials_interfaces::action::Position_FeedbackMessage & msg)
{
  return action_tutorials_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_tutorials_interfaces::action::Position_FeedbackMessage>()
{
  return "action_tutorials_interfaces::action::Position_FeedbackMessage";
}

template<>
inline const char * name<action_tutorials_interfaces::action::Position_FeedbackMessage>()
{
  return "action_tutorials_interfaces/action/Position_FeedbackMessage";
}

template<>
struct has_fixed_size<action_tutorials_interfaces::action::Position_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<action_tutorials_interfaces::action::Position_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_tutorials_interfaces::action::Position_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<action_tutorials_interfaces::action::Position_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_tutorials_interfaces::action::Position_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<action_tutorials_interfaces::action::Position>
  : std::true_type
{
};

template<>
struct is_action_goal<action_tutorials_interfaces::action::Position_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<action_tutorials_interfaces::action::Position_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<action_tutorials_interfaces::action::Position_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ACTION_TUTORIALS_INTERFACES__ACTION__DETAIL__POSITION__TRAITS_HPP_
