// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from marvelmind_ros2_msgs:srv/MarvelmindAPI.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace marvelmind_ros2_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MarvelmindAPI_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) marvelmind_ros2_msgs::srv::MarvelmindAPI_Request(_init);
}

void MarvelmindAPI_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request *>(message_memory);
  typed_message->~MarvelmindAPI_Request();
}

size_t size_function__MarvelmindAPI_Request__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MarvelmindAPI_Request__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__MarvelmindAPI_Request__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__MarvelmindAPI_Request__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__MarvelmindAPI_Request__request(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__MarvelmindAPI_Request__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__MarvelmindAPI_Request__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__MarvelmindAPI_Request__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MarvelmindAPI_Request_message_member_array[2] = {
  {
    "command_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Request, command_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Request, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__MarvelmindAPI_Request__request,  // size() function pointer
    get_const_function__MarvelmindAPI_Request__request,  // get_const(index) function pointer
    get_function__MarvelmindAPI_Request__request,  // get(index) function pointer
    fetch_function__MarvelmindAPI_Request__request,  // fetch(index, &value) function pointer
    assign_function__MarvelmindAPI_Request__request,  // assign(index, value) function pointer
    resize_function__MarvelmindAPI_Request__request  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MarvelmindAPI_Request_message_members = {
  "marvelmind_ros2_msgs::srv",  // message namespace
  "MarvelmindAPI_Request",  // message name
  2,  // number of fields
  sizeof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Request),
  MarvelmindAPI_Request_message_member_array,  // message members
  MarvelmindAPI_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MarvelmindAPI_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MarvelmindAPI_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MarvelmindAPI_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace marvelmind_ros2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<marvelmind_ros2_msgs::srv::MarvelmindAPI_Request>()
{
  return &::marvelmind_ros2_msgs::srv::rosidl_typesupport_introspection_cpp::MarvelmindAPI_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, marvelmind_ros2_msgs, srv, MarvelmindAPI_Request)() {
  return &::marvelmind_ros2_msgs::srv::rosidl_typesupport_introspection_cpp::MarvelmindAPI_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace marvelmind_ros2_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MarvelmindAPI_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) marvelmind_ros2_msgs::srv::MarvelmindAPI_Response(_init);
}

void MarvelmindAPI_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response *>(message_memory);
  typed_message->~MarvelmindAPI_Response();
}

size_t size_function__MarvelmindAPI_Response__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MarvelmindAPI_Response__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__MarvelmindAPI_Response__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__MarvelmindAPI_Response__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__MarvelmindAPI_Response__response(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__MarvelmindAPI_Response__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__MarvelmindAPI_Response__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__MarvelmindAPI_Response__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MarvelmindAPI_Response_message_member_array[3] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Response, error_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Response, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__MarvelmindAPI_Response__response,  // size() function pointer
    get_const_function__MarvelmindAPI_Response__response,  // get_const(index) function pointer
    get_function__MarvelmindAPI_Response__response,  // get(index) function pointer
    fetch_function__MarvelmindAPI_Response__response,  // fetch(index, &value) function pointer
    assign_function__MarvelmindAPI_Response__response,  // assign(index, value) function pointer
    resize_function__MarvelmindAPI_Response__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MarvelmindAPI_Response_message_members = {
  "marvelmind_ros2_msgs::srv",  // message namespace
  "MarvelmindAPI_Response",  // message name
  3,  // number of fields
  sizeof(marvelmind_ros2_msgs::srv::MarvelmindAPI_Response),
  MarvelmindAPI_Response_message_member_array,  // message members
  MarvelmindAPI_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MarvelmindAPI_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MarvelmindAPI_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MarvelmindAPI_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace marvelmind_ros2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<marvelmind_ros2_msgs::srv::MarvelmindAPI_Response>()
{
  return &::marvelmind_ros2_msgs::srv::rosidl_typesupport_introspection_cpp::MarvelmindAPI_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, marvelmind_ros2_msgs, srv, MarvelmindAPI_Response)() {
  return &::marvelmind_ros2_msgs::srv::rosidl_typesupport_introspection_cpp::MarvelmindAPI_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "marvelmind_ros2_msgs/srv/detail/marvelmind_api__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace marvelmind_ros2_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MarvelmindAPI_service_members = {
  "marvelmind_ros2_msgs::srv",  // service namespace
  "MarvelmindAPI",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<marvelmind_ros2_msgs::srv::MarvelmindAPI>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t MarvelmindAPI_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MarvelmindAPI_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace marvelmind_ros2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<marvelmind_ros2_msgs::srv::MarvelmindAPI>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::marvelmind_ros2_msgs::srv::rosidl_typesupport_introspection_cpp::MarvelmindAPI_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::marvelmind_ros2_msgs::srv::MarvelmindAPI_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, marvelmind_ros2_msgs, srv, MarvelmindAPI)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<marvelmind_ros2_msgs::srv::MarvelmindAPI>();
}

#ifdef __cplusplus
}
#endif
