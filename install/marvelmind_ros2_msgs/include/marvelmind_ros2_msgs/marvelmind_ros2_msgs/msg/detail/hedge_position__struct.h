// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:msg/HedgePosition.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/HedgePosition in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__msg__HedgePosition
{
  int64_t timestamp_ms;
  double x_m;
  double y_m;
  double z_m;
  uint8_t flags;
} marvelmind_ros2_msgs__msg__HedgePosition;

// Struct for a sequence of marvelmind_ros2_msgs__msg__HedgePosition.
typedef struct marvelmind_ros2_msgs__msg__HedgePosition__Sequence
{
  marvelmind_ros2_msgs__msg__HedgePosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__msg__HedgePosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_POSITION__STRUCT_H_
