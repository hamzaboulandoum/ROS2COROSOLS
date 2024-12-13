// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ImuData in the package custom_interfaces.
typedef struct custom_interfaces__msg__ImuData
{
  float gyroscope_x;
  float gyroscope_y;
  float gyroscope_z;
  float accelerometer_x;
  float accelerometer_y;
  float accelerometer_z;
  float magnetometer_x;
  float magnetometer_y;
  float magnetometer_z;
  float roll_speed;
  float pitch_speed;
  float heading_speed;
  float roll;
  float pitch;
  float heading;
  float q1;
  float q2;
  float q3;
  float q4;
  float temperature;
  float pressure;
  float pressure_temp;
  int32_t timestamp;
} custom_interfaces__msg__ImuData;

// Struct for a sequence of custom_interfaces__msg__ImuData.
typedef struct custom_interfaces__msg__ImuData__Sequence
{
  custom_interfaces__msg__ImuData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__ImuData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__IMU_DATA__STRUCT_H_
