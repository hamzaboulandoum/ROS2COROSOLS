// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindUserData.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MarvelmindUserData in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__msg__MarvelmindUserData
{
  int64_t timestamp_ms;
  rosidl_runtime_c__uint8__Sequence data;
} marvelmind_ros2_msgs__msg__MarvelmindUserData;

// Struct for a sequence of marvelmind_ros2_msgs__msg__MarvelmindUserData.
typedef struct marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence
{
  marvelmind_ros2_msgs__msg__MarvelmindUserData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__STRUCT_H_
