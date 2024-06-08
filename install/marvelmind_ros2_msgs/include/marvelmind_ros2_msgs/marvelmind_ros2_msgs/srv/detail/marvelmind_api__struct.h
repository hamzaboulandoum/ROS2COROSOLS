// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from marvelmind_ros2_msgs:srv/MarvelmindAPI.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__STRUCT_H_
#define MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MarvelmindAPI in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__srv__MarvelmindAPI_Request
{
  int64_t command_id;
  rosidl_runtime_c__uint8__Sequence request;
} marvelmind_ros2_msgs__srv__MarvelmindAPI_Request;

// Struct for a sequence of marvelmind_ros2_msgs__srv__MarvelmindAPI_Request.
typedef struct marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__Sequence
{
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MarvelmindAPI in the package marvelmind_ros2_msgs.
typedef struct marvelmind_ros2_msgs__srv__MarvelmindAPI_Response
{
  bool success;
  int32_t error_code;
  rosidl_runtime_c__uint8__Sequence response;
} marvelmind_ros2_msgs__srv__MarvelmindAPI_Response;

// Struct for a sequence of marvelmind_ros2_msgs__srv__MarvelmindAPI_Response.
typedef struct marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__Sequence
{
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__SRV__DETAIL__MARVELMIND_API__STRUCT_H_
