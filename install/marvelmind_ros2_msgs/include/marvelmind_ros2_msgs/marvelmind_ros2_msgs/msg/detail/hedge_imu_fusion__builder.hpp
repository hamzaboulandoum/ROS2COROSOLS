// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuFusion.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_FUSION__BUILDER_HPP_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_FUSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_fusion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_HedgeImuFusion_az
{
public:
  explicit Init_HedgeImuFusion_az(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion az(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_az_type arg)
  {
    msg_.az = std::move(arg);
    return std::move(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_ay
{
public:
  explicit Init_HedgeImuFusion_ay(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_az ay(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_ay_type arg)
  {
    msg_.ay = std::move(arg);
    return Init_HedgeImuFusion_az(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_ax
{
public:
  explicit Init_HedgeImuFusion_ax(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_ay ax(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_ax_type arg)
  {
    msg_.ax = std::move(arg);
    return Init_HedgeImuFusion_ay(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_vz
{
public:
  explicit Init_HedgeImuFusion_vz(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_ax vz(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return Init_HedgeImuFusion_ax(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_vy
{
public:
  explicit Init_HedgeImuFusion_vy(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_vz vy(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_HedgeImuFusion_vz(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_vx
{
public:
  explicit Init_HedgeImuFusion_vx(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_vy vx(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_HedgeImuFusion_vy(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_qz
{
public:
  explicit Init_HedgeImuFusion_qz(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_vx qz(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_qz_type arg)
  {
    msg_.qz = std::move(arg);
    return Init_HedgeImuFusion_vx(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_qy
{
public:
  explicit Init_HedgeImuFusion_qy(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_qz qy(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_qy_type arg)
  {
    msg_.qy = std::move(arg);
    return Init_HedgeImuFusion_qz(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_qx
{
public:
  explicit Init_HedgeImuFusion_qx(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_qy qx(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_qx_type arg)
  {
    msg_.qx = std::move(arg);
    return Init_HedgeImuFusion_qy(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_qw
{
public:
  explicit Init_HedgeImuFusion_qw(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_qx qw(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_qw_type arg)
  {
    msg_.qw = std::move(arg);
    return Init_HedgeImuFusion_qx(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_z_m
{
public:
  explicit Init_HedgeImuFusion_z_m(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_qw z_m(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_z_m_type arg)
  {
    msg_.z_m = std::move(arg);
    return Init_HedgeImuFusion_qw(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_y_m
{
public:
  explicit Init_HedgeImuFusion_y_m(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_z_m y_m(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_HedgeImuFusion_z_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_x_m
{
public:
  explicit Init_HedgeImuFusion_x_m(::marvelmind_ros2_msgs::msg::HedgeImuFusion & msg)
  : msg_(msg)
  {}
  Init_HedgeImuFusion_y_m x_m(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_HedgeImuFusion_y_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

class Init_HedgeImuFusion_timestamp_ms
{
public:
  Init_HedgeImuFusion_timestamp_ms()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HedgeImuFusion_x_m timestamp_ms(::marvelmind_ros2_msgs::msg::HedgeImuFusion::_timestamp_ms_type arg)
  {
    msg_.timestamp_ms = std::move(arg);
    return Init_HedgeImuFusion_x_m(msg_);
  }

private:
  ::marvelmind_ros2_msgs::msg::HedgeImuFusion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::marvelmind_ros2_msgs::msg::HedgeImuFusion>()
{
  return marvelmind_ros2_msgs::msg::builder::Init_HedgeImuFusion_timestamp_ms();
}

}  // namespace marvelmind_ros2_msgs

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_FUSION__BUILDER_HPP_
