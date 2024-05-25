// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Target in the package custom_interfaces.
typedef struct custom_interfaces__msg__Target
{
  int32_t x;
  int32_t y;
  int32_t r;
  int32_t airbrush;
} custom_interfaces__msg__Target;

// Struct for a sequence of custom_interfaces__msg__Target.
typedef struct custom_interfaces__msg__Target__Sequence
{
  custom_interfaces__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_
