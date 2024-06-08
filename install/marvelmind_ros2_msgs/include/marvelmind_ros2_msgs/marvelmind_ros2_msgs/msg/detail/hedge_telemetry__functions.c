// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from marvelmind_ros2_msgs:msg/HedgeTelemetry.idl
// generated code does not contain a copyright notice
#include "marvelmind_ros2_msgs/msg/detail/hedge_telemetry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
marvelmind_ros2_msgs__msg__HedgeTelemetry__init(marvelmind_ros2_msgs__msg__HedgeTelemetry * msg)
{
  if (!msg) {
    return false;
  }
  // battery_voltage
  // rssi_dbm
  return true;
}

void
marvelmind_ros2_msgs__msg__HedgeTelemetry__fini(marvelmind_ros2_msgs__msg__HedgeTelemetry * msg)
{
  if (!msg) {
    return;
  }
  // battery_voltage
  // rssi_dbm
}

bool
marvelmind_ros2_msgs__msg__HedgeTelemetry__are_equal(const marvelmind_ros2_msgs__msg__HedgeTelemetry * lhs, const marvelmind_ros2_msgs__msg__HedgeTelemetry * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // rssi_dbm
  if (lhs->rssi_dbm != rhs->rssi_dbm) {
    return false;
  }
  return true;
}

bool
marvelmind_ros2_msgs__msg__HedgeTelemetry__copy(
  const marvelmind_ros2_msgs__msg__HedgeTelemetry * input,
  marvelmind_ros2_msgs__msg__HedgeTelemetry * output)
{
  if (!input || !output) {
    return false;
  }
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // rssi_dbm
  output->rssi_dbm = input->rssi_dbm;
  return true;
}

marvelmind_ros2_msgs__msg__HedgeTelemetry *
marvelmind_ros2_msgs__msg__HedgeTelemetry__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  marvelmind_ros2_msgs__msg__HedgeTelemetry * msg = (marvelmind_ros2_msgs__msg__HedgeTelemetry *)allocator.allocate(sizeof(marvelmind_ros2_msgs__msg__HedgeTelemetry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(marvelmind_ros2_msgs__msg__HedgeTelemetry));
  bool success = marvelmind_ros2_msgs__msg__HedgeTelemetry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
marvelmind_ros2_msgs__msg__HedgeTelemetry__destroy(marvelmind_ros2_msgs__msg__HedgeTelemetry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    marvelmind_ros2_msgs__msg__HedgeTelemetry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__init(marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  marvelmind_ros2_msgs__msg__HedgeTelemetry * data = NULL;

  if (size) {
    data = (marvelmind_ros2_msgs__msg__HedgeTelemetry *)allocator.zero_allocate(size, sizeof(marvelmind_ros2_msgs__msg__HedgeTelemetry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = marvelmind_ros2_msgs__msg__HedgeTelemetry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        marvelmind_ros2_msgs__msg__HedgeTelemetry__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__fini(marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      marvelmind_ros2_msgs__msg__HedgeTelemetry__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence *
marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * array = (marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence *)allocator.allocate(sizeof(marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__destroy(marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__are_equal(const marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * lhs, const marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!marvelmind_ros2_msgs__msg__HedgeTelemetry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence__copy(
  const marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * input,
  marvelmind_ros2_msgs__msg__HedgeTelemetry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(marvelmind_ros2_msgs__msg__HedgeTelemetry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    marvelmind_ros2_msgs__msg__HedgeTelemetry * data =
      (marvelmind_ros2_msgs__msg__HedgeTelemetry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!marvelmind_ros2_msgs__msg__HedgeTelemetry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          marvelmind_ros2_msgs__msg__HedgeTelemetry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!marvelmind_ros2_msgs__msg__HedgeTelemetry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
