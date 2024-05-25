// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_Command_airbrush
{
public:
  explicit Init_Command_airbrush(::custom_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::Command airbrush(::custom_interfaces::msg::Command::_airbrush_type arg)
  {
    msg_.airbrush = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::Command msg_;
};

class Init_Command_vr
{
public:
  explicit Init_Command_vr(::custom_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_airbrush vr(::custom_interfaces::msg::Command::_vr_type arg)
  {
    msg_.vr = std::move(arg);
    return Init_Command_airbrush(msg_);
  }

private:
  ::custom_interfaces::msg::Command msg_;
};

class Init_Command_vy
{
public:
  explicit Init_Command_vy(::custom_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_vr vy(::custom_interfaces::msg::Command::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_Command_vr(msg_);
  }

private:
  ::custom_interfaces::msg::Command msg_;
};

class Init_Command_vx
{
public:
  Init_Command_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_vy vx(::custom_interfaces::msg::Command::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_Command_vy(msg_);
  }

private:
  ::custom_interfaces::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::Command>()
{
  return custom_interfaces::msg::builder::Init_Command_vx();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_
