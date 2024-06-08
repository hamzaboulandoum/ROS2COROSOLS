// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:msg/HedgePositionAngle.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/HedgePositionAngle in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__msg__HedgePositionAngle
{
  uint8_t address;
  int64_t timestamp_ms;
  double x_m;
  double y_m;
  double z_m;
  uint8_t flags;
  double angle;
} marvelmind_ros2_msgs__msg__HedgePositionAngle;

// Struct for a sequence of marvelmind_ros2_msgs__msg__HedgePositionAngle.
typedef struct marvelmind_ros2_msgs__msg__HedgePositionAngle__Sequence
{
  marvelmind_ros2_msgs__msg__HedgePositionAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__msg__HedgePositionAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION_ANGLE__STRUCT_H_
