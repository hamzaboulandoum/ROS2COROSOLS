// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_Target_airbrush
{
public:
  explicit Init_Target_airbrush(::custom_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::Target airbrush(::custom_interfaces::msg::Target::_airbrush_type arg)
  {
    msg_.airbrush = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::Target msg_;
};

class Init_Target_r
{
public:
  explicit Init_Target_r(::custom_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_airbrush r(::custom_interfaces::msg::Target::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_Target_airbrush(msg_);
  }

private:
  ::custom_interfaces::msg::Target msg_;
};

class Init_Target_y
{
public:
  explicit Init_Target_y(::custom_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_r y(::custom_interfaces::msg::Target::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Target_r(msg_);
  }

private:
  ::custom_interfaces::msg::Target msg_;
};

class Init_Target_x
{
public:
  Init_Target_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_y x(::custom_interfaces::msg::Target::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Target_y(msg_);
  }

private:
  ::custom_interfaces::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::Target>()
{
  return custom_interfaces::msg::builder::Init_Target_x();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
