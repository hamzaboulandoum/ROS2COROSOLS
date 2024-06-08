// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuRaw.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_raw__rosidl_typesupport_introspection_c.h"
#include "marvelmind_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_raw__functions.h"
#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_raw__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  marvelmind_ros2_msgs__msg__HedgeImuRaw__init(message_memory);
}

void marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_fini_function(void * message_memory)
{
  marvelmind_ros2_msgs__msg__HedgeImuRaw__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_member_array[10] = {
  {
    "timestamp_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, timestamp_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, acc_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, acc_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, acc_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyro_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, gyro_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyro_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, gyro_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyro_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, gyro_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compass_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, compass_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compass_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, compass_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compass_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__HedgeImuRaw, compass_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_members = {
  "marvelmind_ros2_msgs__msg",  // message namespace
  "HedgeImuRaw",  // message name
  10,  // number of fields
  sizeof(marvelmind_ros2_msgs__msg__HedgeImuRaw),
  marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_member_array,  // message members
  marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_init_function,  // function to initialize message memory (memory has to be allocated)
  marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_type_support_handle = {
  0,
  &marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_marvelmind_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, msg, HedgeImuRaw)() {
  if (!marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_type_support_handle.typesupport_identifier) {
    marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &marvelmind_ros2_msgs__msg__HedgeImuRaw__rosidl_typesupport_introspection_c__HedgeImuRaw_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
