// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:msg/HedgeTelemetry.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/HedgeTelemetry in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__msg__HedgeTelemetry
{
  double battery_voltage;
  int8_t rssi_dbm;
} marvelmind_ros2_msgs__msg__HedgeTelemetry;

// Struct for a sequence of marvelmind_ros2_msgs__msg__HedgeTelemetry.
typedef struct marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence
{
  marvelmind_ros2_msgs__msg__HedgeTelemetry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_TELEMETRY__STRUCT_H_
