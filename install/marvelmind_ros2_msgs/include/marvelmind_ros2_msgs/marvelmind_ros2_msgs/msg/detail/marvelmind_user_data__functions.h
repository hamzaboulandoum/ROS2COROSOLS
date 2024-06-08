// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from marvelmind_ros2_msgs:msg/MarvelmindUserData.idl
// generated code does not contain a copyright notice

#ifndef MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__FUNCTIONS_H_
#define MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "marvelmind_ros2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "marvelmind_ros2_msgs/msg/detail/marvelmind_user_data__struct.h"

/// Initialize msg/MarvelmindUserData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * marvelmind_ros2_msgs__msg__MarvelmindUserData
 * )) before or use
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
bool
marvelmind_ros2_msgs__msg__MarvelmindUserData__init(marvelmind_ros2_msgs__msg__MarvelmindUserData * msg);

/// Finalize msg/MarvelmindUserData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
void
marvelmind_ros2_msgs__msg__MarvelmindUserData__fini(marvelmind_ros2_msgs__msg__MarvelmindUserData * msg);

/// Create msg/MarvelmindUserData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
marvelmind_ros2_msgs__msg__MarvelmindUserData *
marvelmind_ros2_msgs__msg__MarvelmindUserData__create();

/// Destroy msg/MarvelmindUserData message.
/**
 * It calls
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
void
marvelmind_ros2_msgs__msg__MarvelmindUserData__destroy(marvelmind_ros2_msgs__msg__MarvelmindUserData * msg);

/// Check for msg/MarvelmindUserData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
bool
marvelmind_ros2_msgs__msg__MarvelmindUserData__are_equal(const marvelmind_ros2_msgs__msg__MarvelmindUserData * lhs, const marvelmind_ros2_msgs__msg__MarvelmindUserData * rhs);

/// Copy a msg/MarvelmindUserData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
bool
marvelmind_ros2_msgs__msg__MarvelmindUserData__copy(
  const marvelmind_ros2_msgs__msg__MarvelmindUserData * input,
  marvelmind_ros2_msgs__msg__MarvelmindUserData * output);

/// Initialize array of msg/MarvelmindUserData messages.
/**
 * It allocates the memory for the number of elements and calls
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
bool
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__init(marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * array, size_t size);

/// Finalize array of msg/MarvelmindUserData messages.
/**
 * It calls
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
void
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__fini(marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * array);

/// Create array of msg/MarvelmindUserData messages.
/**
 * It allocates the memory for the array and calls
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence *
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__create(size_t size);

/// Destroy array of msg/MarvelmindUserData messages.
/**
 * It calls
 * marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
void
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__destroy(marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * array);

/// Check for msg/MarvelmindUserData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
bool
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__are_equal(const marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * lhs, const marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * rhs);

/// Copy an array of msg/MarvelmindUserData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_marvelmind_ros2_msgs
bool
marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence__copy(
  const marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * input,
  marvelmind_ros2_msgs__msg__MarvelmindUserData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MARVELMIND_ROS2_MSGS__MSG__DETAIL__MARVELMIND_USER_DATA__FUNCTIONS_H_
