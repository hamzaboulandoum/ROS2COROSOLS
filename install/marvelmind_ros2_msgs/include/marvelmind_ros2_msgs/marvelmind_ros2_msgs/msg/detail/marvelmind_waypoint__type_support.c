// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindWaypoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "marvelmind_ros2_msgs/msg/detail/marvelmind_waypoint__rosidl_typesupport_introspection_c.h"
#include "marvelmind_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "marvelmind_ros2_msgs/msg/detail/marvelmind_waypoint__functions.h"
#include "marvelmind_ros2_msgs/msg/detail/marvelmind_waypoint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  marvelmind_ros2_msgs__msg__MarvelmindWaypoint__init(message_memory);
}

void marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_fini_function(void * message_memory)
{
  marvelmind_ros2_msgs__msg__MarvelmindWaypoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_member_array[6] = {
  {
    "total_items",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint, total_items),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "item_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint, item_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "movement_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint, movement_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint, param1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint, param2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint, param3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_members = {
  "marvelmind_ros2_msgs__msg",  // message namespace
  "MarvelmindWaypoint",  // message name
  6,  // number of fields
  sizeof(marvelmind_ros2_msgs__msg__MarvelmindWaypoint),
  marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_member_array,  // message members
  marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_init_function,  // function to initialize message memory (memory has to be allocated)
  marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_type_support_handle = {
  0,
  &marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_marvelmind_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, msg, MarvelmindWaypoint)() {
  if (!marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_type_support_handle.typesupport_identifier) {
    marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &marvelmind_ros2_msgs__msg__MarvelmindWaypoint__rosidl_typesupport_introspection_c__MarvelmindWaypoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
