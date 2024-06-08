// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from marvelmind_ros2_msgs:msg/BeaconDistance.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "marvelmind_ros2_msgs/msg/detail/beacon_distance__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace marvelmind_ros2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BeaconDistance_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) marvelmind_ros2_msgs::msg::BeaconDistance(_init);
}

void BeaconDistance_fini_function(void * message_memory)
{
  auto typed_message = static_cast<marvelmind_ros2_msgs::msg::BeaconDistance *>(message_memory);
  typed_message->~BeaconDistance();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BeaconDistance_message_member_array[3] = {
  {
    "address_hedge",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::msg::BeaconDistance, address_hedge),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "address_beacon",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::msg::BeaconDistance, address_beacon),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "distance_m",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(marvelmind_ros2_msgs::msg::BeaconDistance, distance_m),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BeaconDistance_message_members = {
  "marvelmind_ros2_msgs::msg",  // message namespace
  "BeaconDistance",  // message name
  3,  // number of fields
  sizeof(marvelmind_ros2_msgs::msg::BeaconDistance),
  BeaconDistance_message_member_array,  // message members
  BeaconDistance_init_function,  // function to initialize message memory (memory has to be allocated)
  BeaconDistance_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BeaconDistance_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BeaconDistance_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace marvelmind_ros2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<marvelmind_ros2_msgs::msg::BeaconDistance>()
{
  return &::marvelmind_ros2_msgs::msg::rosidl_typesupport_introspection_cpp::BeaconDistance_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, marvelmind_ros2_msgs, msg, BeaconDistance)() {
  return &::marvelmind_ros2_msgs::msg::rosidl_typesupport_introspection_cpp::BeaconDistance_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
