// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_tutorials_interfaces:action/Position.idl
// generated code does not contain a copyright notice

#ifndef ACTION_TUTORIALS_INTERFACES__ACTION__DETAIL__POSITION__BUILDER_HPP_
#define ACTION_TUTORIALS_INTERFACES__ACTION__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_tutorials_interfaces/action/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_Goal_a
{
public:
  explicit Init_Position_Goal_a(::action_tutorials_interfaces::action::Position_Goal & msg)
  : msg_(msg)
  {}
  ::action_tutorials_interfaces::action::Position_Goal a(::action_tutorials_interfaces::action::Position_Goal::_a_type arg)
  {
    msg_.a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_Goal msg_;
};

class Init_Position_Goal_r
{
public:
  explicit Init_Position_Goal_r(::action_tutorials_interfaces::action::Position_Goal & msg)
  : msg_(msg)
  {}
  Init_Position_Goal_a r(::action_tutorials_interfaces::action::Position_Goal::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_Position_Goal_a(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_Goal msg_;
};

class Init_Position_Goal_y
{
public:
  explicit Init_Position_Goal_y(::action_tutorials_interfaces::action::Position_Goal & msg)
  : msg_(msg)
  {}
  Init_Position_Goal_r y(::action_tutorials_interfaces::action::Position_Goal::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Position_Goal_r(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_Goal msg_;
};

class Init_Position_Goal_x
{
public:
  Init_Position_Goal_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_Goal_y x(::action_tutorials_interfaces::action::Position_Goal::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Position_Goal_y(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_Goal>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_Goal_x();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_Result_precision
{
public:
  Init_Position_Result_precision()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_tutorials_interfaces::action::Position_Result precision(::action_tutorials_interfaces::action::Position_Result::_precision_type arg)
  {
    msg_.precision = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_Result>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_Result_precision();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_Feedback_precision_log
{
public:
  Init_Position_Feedback_precision_log()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_tutorials_interfaces::action::Position_Feedback precision_log(::action_tutorials_interfaces::action::Position_Feedback::_precision_log_type arg)
  {
    msg_.precision_log = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_Feedback>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_Feedback_precision_log();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_SendGoal_Request_goal
{
public:
  explicit Init_Position_SendGoal_Request_goal(::action_tutorials_interfaces::action::Position_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_tutorials_interfaces::action::Position_SendGoal_Request goal(::action_tutorials_interfaces::action::Position_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_SendGoal_Request msg_;
};

class Init_Position_SendGoal_Request_goal_id
{
public:
  Init_Position_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_SendGoal_Request_goal goal_id(::action_tutorials_interfaces::action::Position_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Position_SendGoal_Request_goal(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_SendGoal_Request>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_SendGoal_Request_goal_id();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_SendGoal_Response_stamp
{
public:
  explicit Init_Position_SendGoal_Response_stamp(::action_tutorials_interfaces::action::Position_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_tutorials_interfaces::action::Position_SendGoal_Response stamp(::action_tutorials_interfaces::action::Position_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_SendGoal_Response msg_;
};

class Init_Position_SendGoal_Response_accepted
{
public:
  Init_Position_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_SendGoal_Response_stamp accepted(::action_tutorials_interfaces::action::Position_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Position_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_SendGoal_Response>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_SendGoal_Response_accepted();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_GetResult_Request_goal_id
{
public:
  Init_Position_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_tutorials_interfaces::action::Position_GetResult_Request goal_id(::action_tutorials_interfaces::action::Position_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_GetResult_Request>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_GetResult_Request_goal_id();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_GetResult_Response_result
{
public:
  explicit Init_Position_GetResult_Response_result(::action_tutorials_interfaces::action::Position_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_tutorials_interfaces::action::Position_GetResult_Response result(::action_tutorials_interfaces::action::Position_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_GetResult_Response msg_;
};

class Init_Position_GetResult_Response_status
{
public:
  Init_Position_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_GetResult_Response_result status(::action_tutorials_interfaces::action::Position_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Position_GetResult_Response_result(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_GetResult_Response>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_GetResult_Response_status();
}

}  // namespace action_tutorials_interfaces


namespace action_tutorials_interfaces
{

namespace action
{

namespace builder
{

class Init_Position_FeedbackMessage_feedback
{
public:
  explicit Init_Position_FeedbackMessage_feedback(::action_tutorials_interfaces::action::Position_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_tutorials_interfaces::action::Position_FeedbackMessage feedback(::action_tutorials_interfaces::action::Position_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_FeedbackMessage msg_;
};

class Init_Position_FeedbackMessage_goal_id
{
public:
  Init_Position_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_FeedbackMessage_feedback goal_id(::action_tutorials_interfaces::action::Position_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Position_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_tutorials_interfaces::action::Position_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_tutorials_interfaces::action::Position_FeedbackMessage>()
{
  return action_tutorials_interfaces::action::builder::Init_Position_FeedbackMessage_goal_id();
}

}  // namespace action_tutorials_interfaces

#endif  // ACTION_TUTORIALS_INTERFACES__ACTION__DETAIL__POSITION__BUILDER_HPP_
