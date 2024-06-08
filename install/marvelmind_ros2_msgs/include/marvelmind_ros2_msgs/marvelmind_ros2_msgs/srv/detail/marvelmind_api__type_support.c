// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from marvelmind_ros2_msgs:srv/MarvelmindAPI.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__rosidl_typesupport_introspection_c.h"
#include "marvelmind_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__functions.h"
#include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.h"


// Include directives for member types
// Member `request`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__init(message_memory);
}

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_fini_function(void * message_memory)
{
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__fini(message_memory);
}

size_t marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__size_function__MarvelmindAPI_Request__request(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__get_const_function__MarvelmindAPI_Request__request(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__get_function__MarvelmindAPI_Request__request(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__fetch_function__MarvelmindAPI_Request__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__get_const_function__MarvelmindAPI_Request__request(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__assign_function__MarvelmindAPI_Request__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__get_function__MarvelmindAPI_Request__request(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__resize_function__MarvelmindAPI_Request__request(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_member_array[2] = {
  {
    "command_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Request, command_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Request, request),  // bytes offset in struct
    NULL,  // default value
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__size_function__MarvelmindAPI_Request__request,  // size() function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__get_const_function__MarvelmindAPI_Request__request,  // get_const(index) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__get_function__MarvelmindAPI_Request__request,  // get(index) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__fetch_function__MarvelmindAPI_Request__request,  // fetch(index, &value) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__assign_function__MarvelmindAPI_Request__request,  // assign(index, value) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__resize_function__MarvelmindAPI_Request__request  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_members = {
  "marvelmind_ros2_msgs__srv",  // message namespace
  "MarvelmindAPI_Request",  // message name
  2,  // number of fields
  sizeof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Request),
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_member_array,  // message members
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_type_support_handle = {
  0,
  &marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_marvelmind_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI_Request)() {
  if (!marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_type_support_handle.typesupport_identifier) {
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &marvelmind_ros2_msgs__srv__MarvelmindAPI_Request__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__rosidl_typesupport_introspection_c.h"
// already included above
// #include "marvelmind_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__functions.h"
// already included above
// #include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.h"


// Include directives for member types
// Member `response`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__init(message_memory);
}

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_fini_function(void * message_memory)
{
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__fini(message_memory);
}

size_t marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__size_function__MarvelmindAPI_Response__response(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__get_const_function__MarvelmindAPI_Response__response(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__get_function__MarvelmindAPI_Response__response(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__fetch_function__MarvelmindAPI_Response__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__get_const_function__MarvelmindAPI_Response__response(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__assign_function__MarvelmindAPI_Response__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__get_function__MarvelmindAPI_Response__response(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__resize_function__MarvelmindAPI_Response__response(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Response, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Response, response),  // bytes offset in struct
    NULL,  // default value
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__size_function__MarvelmindAPI_Response__response,  // size() function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__get_const_function__MarvelmindAPI_Response__response,  // get_const(index) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__get_function__MarvelmindAPI_Response__response,  // get(index) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__fetch_function__MarvelmindAPI_Response__response,  // fetch(index, &value) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__assign_function__MarvelmindAPI_Response__response,  // assign(index, value) function pointer
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__resize_function__MarvelmindAPI_Response__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_members = {
  "marvelmind_ros2_msgs__srv",  // message namespace
  "MarvelmindAPI_Response",  // message name
  3,  // number of fields
  sizeof(marvelmind_ros2_msgs__srv__MarvelmindAPI_Response),
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_member_array,  // message members
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_type_support_handle = {
  0,
  &marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_marvelmind_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI_Response)() {
  if (!marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_type_support_handle.typesupport_identifier) {
    marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &marvelmind_ros2_msgs__srv__MarvelmindAPI_Response__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "marvelmind_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_members = {
  "marvelmind_ros2_msgs__srv",  // service namespace
  "MarvelmindAPI",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_Request_message_type_support_handle,
  NULL  // response message
  // marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_Response_message_type_support_handle
};

static rosidl_service_type_support_t marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_type_support_handle = {
  0,
  &marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_marvelmind_ros2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI)() {
  if (!marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_type_support_handle.typesupport_identifier) {
    marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, marvelmind_ros2_msgs, srv, MarvelmindAPI_Response)()->data;
  }

  return &marvelmind_ros2_msgs__srv__detail__marvelmind_api__rosidl_typesupport_introspection_c__MarvelmindAPI_service_type_support_handle;
}
