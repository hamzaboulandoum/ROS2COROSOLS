// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/ImuData.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/imu_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_interfaces__msg__ImuData__init(custom_interfaces__msg__ImuData * msg)
{
  if (!msg) {
    return false;
  }
  // gyroscope_x
  // gyroscope_y
  // gyroscope_z
  // accelerometer_x
  // accelerometer_y
  // accelerometer_z
  // magnetometer_x
  // magnetometer_y
  // magnetometer_z
  // roll_speed
  // pitch_speed
  // heading_speed
  // roll
  // pitch
  // heading
  // q1
  // q2
  // q3
  // q4
  // temperature
  // pressure
  // pressure_temp
  // timestamp
  return true;
}

void
custom_interfaces__msg__ImuData__fini(custom_interfaces__msg__ImuData * msg)
{
  if (!msg) {
    return;
  }
  // gyroscope_x
  // gyroscope_y
  // gyroscope_z
  // accelerometer_x
  // accelerometer_y
  // accelerometer_z
  // magnetometer_x
  // magnetometer_y
  // magnetometer_z
  // roll_speed
  // pitch_speed
  // heading_speed
  // roll
  // pitch
  // heading
  // q1
  // q2
  // q3
  // q4
  // temperature
  // pressure
  // pressure_temp
  // timestamp
}

bool
custom_interfaces__msg__ImuData__are_equal(const custom_interfaces__msg__ImuData * lhs, const custom_interfaces__msg__ImuData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // gyroscope_x
  if (lhs->gyroscope_x != rhs->gyroscope_x) {
    return false;
  }
  // gyroscope_y
  if (lhs->gyroscope_y != rhs->gyroscope_y) {
    return false;
  }
  // gyroscope_z
  if (lhs->gyroscope_z != rhs->gyroscope_z) {
    return false;
  }
  // accelerometer_x
  if (lhs->accelerometer_x != rhs->accelerometer_x) {
    return false;
  }
  // accelerometer_y
  if (lhs->accelerometer_y != rhs->accelerometer_y) {
    return false;
  }
  // accelerometer_z
  if (lhs->accelerometer_z != rhs->accelerometer_z) {
    return false;
  }
  // magnetometer_x
  if (lhs->magnetometer_x != rhs->magnetometer_x) {
    return false;
  }
  // magnetometer_y
  if (lhs->magnetometer_y != rhs->magnetometer_y) {
    return false;
  }
  // magnetometer_z
  if (lhs->magnetometer_z != rhs->magnetometer_z) {
    return false;
  }
  // roll_speed
  if (lhs->roll_speed != rhs->roll_speed) {
    return false;
  }
  // pitch_speed
  if (lhs->pitch_speed != rhs->pitch_speed) {
    return false;
  }
  // heading_speed
  if (lhs->heading_speed != rhs->heading_speed) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // q1
  if (lhs->q1 != rhs->q1) {
    return false;
  }
  // q2
  if (lhs->q2 != rhs->q2) {
    return false;
  }
  // q3
  if (lhs->q3 != rhs->q3) {
    return false;
  }
  // q4
  if (lhs->q4 != rhs->q4) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // pressure
  if (lhs->pressure != rhs->pressure) {
    return false;
  }
  // pressure_temp
  if (lhs->pressure_temp != rhs->pressure_temp) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__ImuData__copy(
  const custom_interfaces__msg__ImuData * input,
  custom_interfaces__msg__ImuData * output)
{
  if (!input || !output) {
    return false;
  }
  // gyroscope_x
  output->gyroscope_x = input->gyroscope_x;
  // gyroscope_y
  output->gyroscope_y = input->gyroscope_y;
  // gyroscope_z
  output->gyroscope_z = input->gyroscope_z;
  // accelerometer_x
  output->accelerometer_x = input->accelerometer_x;
  // accelerometer_y
  output->accelerometer_y = input->accelerometer_y;
  // accelerometer_z
  output->accelerometer_z = input->accelerometer_z;
  // magnetometer_x
  output->magnetometer_x = input->magnetometer_x;
  // magnetometer_y
  output->magnetometer_y = input->magnetometer_y;
  // magnetometer_z
  output->magnetometer_z = input->magnetometer_z;
  // roll_speed
  output->roll_speed = input->roll_speed;
  // pitch_speed
  output->pitch_speed = input->pitch_speed;
  // heading_speed
  output->heading_speed = input->heading_speed;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // heading
  output->heading = input->heading;
  // q1
  output->q1 = input->q1;
  // q2
  output->q2 = input->q2;
  // q3
  output->q3 = input->q3;
  // q4
  output->q4 = input->q4;
  // temperature
  output->temperature = input->temperature;
  // pressure
  output->pressure = input->pressure;
  // pressure_temp
  output->pressure_temp = input->pressure_temp;
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

custom_interfaces__msg__ImuData *
custom_interfaces__msg__ImuData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__ImuData * msg = (custom_interfaces__msg__ImuData *)allocator.allocate(sizeof(custom_interfaces__msg__ImuData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__ImuData));
  bool success = custom_interfaces__msg__ImuData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__ImuData__destroy(custom_interfaces__msg__ImuData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__ImuData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__ImuData__Sequence__init(custom_interfaces__msg__ImuData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__ImuData * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__ImuData *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__ImuData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__ImuData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__ImuData__fini(&data[i - 1]);
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
custom_interfaces__msg__ImuData__Sequence__fini(custom_interfaces__msg__ImuData__Sequence * array)
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
      custom_interfaces__msg__ImuData__fini(&array->data[i]);
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

custom_interfaces__msg__ImuData__Sequence *
custom_interfaces__msg__ImuData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__ImuData__Sequence * array = (custom_interfaces__msg__ImuData__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__ImuData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__ImuData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__ImuData__Sequence__destroy(custom_interfaces__msg__ImuData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__ImuData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__ImuData__Sequence__are_equal(const custom_interfaces__msg__ImuData__Sequence * lhs, const custom_interfaces__msg__ImuData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__ImuData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__ImuData__Sequence__copy(
  const custom_interfaces__msg__ImuData__Sequence * input,
  custom_interfaces__msg__ImuData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__ImuData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__ImuData * data =
      (custom_interfaces__msg__ImuData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__ImuData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__ImuData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__ImuData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
