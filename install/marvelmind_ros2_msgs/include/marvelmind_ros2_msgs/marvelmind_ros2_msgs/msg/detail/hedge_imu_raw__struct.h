// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuRaw.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/HedgeImuRaw in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__msg__HedgeImuRaw
{
  int64_t timestamp_ms;
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t compass_x;
  int16_t compass_y;
  int16_t compass_z;
} marvelmind_ros2_msgs__msg__HedgeImuRaw;

// Struct for a sequence of marvelmind_ros2_msgs__msg__HedgeImuRaw.
typedef struct marvelmind_ros2_msgs__msg__HedgeImuRaw__Sequence
{
  marvelmind_ros2_msgs__msg__HedgeImuRaw * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__msg__HedgeImuRaw__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__HEDGE_IMU_RAW__STRUCT_H_
