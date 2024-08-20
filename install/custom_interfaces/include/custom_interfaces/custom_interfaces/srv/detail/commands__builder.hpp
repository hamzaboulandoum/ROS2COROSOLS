// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/Commands.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/commands__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Commands_Request_airbrush
{
public:
  explicit Init_Commands_Request_airbrush(::custom_interfaces::srv::Commands_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::Commands_Request airbrush(::custom_interfaces::srv::Commands_Request::_airbrush_type arg)
  {
    msg_.airbrush = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Request msg_;
};

class Init_Commands_Request_steppery
{
public:
  explicit Init_Commands_Request_steppery(::custom_interfaces::srv::Commands_Request & msg)
  : msg_(msg)
  {}
  Init_Commands_Request_airbrush steppery(::custom_interfaces::srv::Commands_Request::_steppery_type arg)
  {
    msg_.steppery = std::move(arg);
    return Init_Commands_Request_airbrush(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Request msg_;
};

class Init_Commands_Request_stepperx
{
public:
  explicit Init_Commands_Request_stepperx(::custom_interfaces::srv::Commands_Request & msg)
  : msg_(msg)
  {}
  Init_Commands_Request_steppery stepperx(::custom_interfaces::srv::Commands_Request::_stepperx_type arg)
  {
    msg_.stepperx = std::move(arg);
    return Init_Commands_Request_steppery(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Request msg_;
};

class Init_Commands_Request_vr
{
public:
  explicit Init_Commands_Request_vr(::custom_interfaces::srv::Commands_Request & msg)
  : msg_(msg)
  {}
  Init_Commands_Request_stepperx vr(::custom_interfaces::srv::Commands_Request::_vr_type arg)
  {
    msg_.vr = std::move(arg);
    return Init_Commands_Request_stepperx(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Request msg_;
};

class Init_Commands_Request_vy
{
public:
  explicit Init_Commands_Request_vy(::custom_interfaces::srv::Commands_Request & msg)
  : msg_(msg)
  {}
  Init_Commands_Request_vr vy(::custom_interfaces::srv::Commands_Request::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_Commands_Request_vr(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Request msg_;
};

class Init_Commands_Request_vx
{
public:
  Init_Commands_Request_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Commands_Request_vy vx(::custom_interfaces::srv::Commands_Request::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_Commands_Request_vy(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::Commands_Request>()
{
  return custom_interfaces::srv::builder::Init_Commands_Request_vx();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Commands_Response_outcome
{
public:
  Init_Commands_Response_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::Commands_Response outcome(::custom_interfaces::srv::Commands_Response::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::Commands_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::Commands_Response>()
{
  return custom_interfaces::srv::builder::Init_Commands_Response_outcome();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__COMMANDS__BUILDER_HPP_
