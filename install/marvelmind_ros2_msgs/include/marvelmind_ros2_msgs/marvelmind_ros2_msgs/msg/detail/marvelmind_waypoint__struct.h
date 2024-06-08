// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MarvelmindWaypoint in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__msg__MarvelmindWaypoint
{
  uint8_t total_items;
  uint8_t item_index;
  uint8_t movement_type;
  int16_t param1;
  int16_t param2;
  int16_t param3;
} marvelmind_ros2_msgs__msg__MarvelmindWaypoint;

// Struct for a sequence of marvelmind_ros2_msgs__msg__MarvelmindWaypoint.
typedef struct marvelmind_ros2_msgs__msg__MarvelmindWaypoint__Sequence
{
  marvelmind_ros2_msgs__msg__MarvelmindWaypoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__msg__MarvelmindWaypoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_WAYPOINT__STRUCT_H_
