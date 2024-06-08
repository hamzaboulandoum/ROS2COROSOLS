// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:srv/MarvelmindAPI.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace srv
{

namespace builder
{

class Init_MarvelmindAPI_Request_request
{
public:
  explicit Init_MarvelmindAPI_Request_request(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request request(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request msg_;
};

class Init_MarvelmindAPI_Request_command_id
{
public:
  Init_MarvelmindAPI_Request_command_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarvelmindAPI_Request_request command_id(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request::_command_id_type arg)
  {
    msg_.command_id = std::move(arg);
    return Init_MarvelmindAPI_Request_request(msg_);
  }

private:
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>()
{
  return marvelmind_ros2_msgs::srv::builder::Init_MarvelmindAPI_Request_command_id();
}

}  // namespace marvelmind_ros2_msgs


namespace marvelmind_ros2_msgs
{

namespace srv
{

namespace builder
{

class Init_MarvelmindAPI_Response_response
{
public:
  explicit Init_MarvelmindAPI_Response_response(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response response(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response msg_;
};

class Init_MarvelmindAPI_Response_error_code
{
public:
  explicit Init_MarvelmindAPI_Response_error_code(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response & msg)
  : msg_(msg)
  {}
  Init_MarvelmindAPI_Response_response error_code(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_MarvelmindAPI_Response_response(msg_);
  }

private:
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response msg_;
};

class Init_MarvelmindAPI_Response_success
{
public:
  Init_MarvelmindAPI_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarvelmindAPI_Response_error_code success(::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MarvelmindAPI_Response_error_code(msg_);
  }

private:
  ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>()
{
  return marvelmind_ros2_msgs::srv::builder::Init_MarvelmindAPI_Response_success();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__BUILDER_HPP_
