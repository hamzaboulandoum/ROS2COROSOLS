// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SerialData in the package custom_interfaces.
typedef struct custom_interfaces__msg__SerialData
{
  double x_speed;
  double y_speed;
  double z_speed;
  double x_accel;
  double y_accel;
  double z_accel;
  double x_gyro;
  double y_gyro;
  double z_gyro;
  double power_voltage;
  double stepper_x;
  double stepper_y;
} custom_interfaces__msg__SerialData;

// Struct for a sequence of custom_interfaces__msg__SerialData.
typedef struct custom_interfaces__msg__SerialData__Sequence
{
  custom_interfaces__msg__SerialData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__SerialData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_H_
