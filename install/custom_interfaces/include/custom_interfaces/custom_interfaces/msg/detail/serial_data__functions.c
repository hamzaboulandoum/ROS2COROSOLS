// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/serial_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_interfaces__msg__SerialData__init(custom_interfaces__msg__SerialData * msg)
{
  if (!msg) {
    return false;
  }
  // x_speed
  // y_speed
  // z_speed
  // x_accel
  // y_accel
  // z_accel
  // heading
  // pitch
  // roll
  // power_voltage
  // stepper_x
  // stepper_y
  return true;
}

void
custom_interfaces__msg__SerialData__fini(custom_interfaces__msg__SerialData * msg)
{
  if (!msg) {
    return;
  }
  // x_speed
  // y_speed
  // z_speed
  // x_accel
  // y_accel
  // z_accel
  // heading
  // pitch
  // roll
  // power_voltage
  // stepper_x
  // stepper_y
}

bool
custom_interfaces__msg__SerialData__are_equal(const custom_interfaces__msg__SerialData * lhs, const custom_interfaces__msg__SerialData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_speed
  if (lhs->x_speed != rhs->x_speed) {
    return false;
  }
  // y_speed
  if (lhs->y_speed != rhs->y_speed) {
    return false;
  }
  // z_speed
  if (lhs->z_speed != rhs->z_speed) {
    return false;
  }
  // x_accel
  if (lhs->x_accel != rhs->x_accel) {
    return false;
  }
  // y_accel
  if (lhs->y_accel != rhs->y_accel) {
    return false;
  }
  // z_accel
  if (lhs->z_accel != rhs->z_accel) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // power_voltage
  if (lhs->power_voltage != rhs->power_voltage) {
    return false;
  }
  // stepper_x
  if (lhs->stepper_x != rhs->stepper_x) {
    return false;
  }
  // stepper_y
  if (lhs->stepper_y != rhs->stepper_y) {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__SerialData__copy(
  const custom_interfaces__msg__SerialData * input,
  custom_interfaces__msg__SerialData * output)
{
  if (!input || !output) {
    return false;
  }
  // x_speed
  output->x_speed = input->x_speed;
  // y_speed
  output->y_speed = input->y_speed;
  // z_speed
  output->z_speed = input->z_speed;
  // x_accel
  output->x_accel = input->x_accel;
  // y_accel
  output->y_accel = input->y_accel;
  // z_accel
  output->z_accel = input->z_accel;
  // heading
  output->heading = input->heading;
  // pitch
  output->pitch = input->pitch;
  // roll
  output->roll = input->roll;
  // power_voltage
  output->power_voltage = input->power_voltage;
  // stepper_x
  output->stepper_x = input->stepper_x;
  // stepper_y
  output->stepper_y = input->stepper_y;
  return true;
}

custom_interfaces__msg__SerialData *
custom_interfaces__msg__SerialData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__SerialData * msg = (custom_interfaces__msg__SerialData *)allocator.allocate(sizeof(custom_interfaces__msg__SerialData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__SerialData));
  bool success = custom_interfaces__msg__SerialData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__SerialData__destroy(custom_interfaces__msg__SerialData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__SerialData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__SerialData__Sequence__init(custom_interfaces__msg__SerialData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__SerialData * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__SerialData *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__SerialData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__SerialData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__SerialData__fini(&data[i - 1]);
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
custom_interfaces__msg__SerialData__Sequence__fini(custom_interfaces__msg__SerialData__Sequence * array)
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
      custom_interfaces__msg__SerialData__fini(&array->data[i]);
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

custom_interfaces__msg__SerialData__Sequence *
custom_interfaces__msg__SerialData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__SerialData__Sequence * array = (custom_interfaces__msg__SerialData__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__SerialData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__SerialData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__SerialData__Sequence__destroy(custom_interfaces__msg__SerialData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__SerialData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__SerialData__Sequence__are_equal(const custom_interfaces__msg__SerialData__Sequence * lhs, const custom_interfaces__msg__SerialData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__SerialData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__SerialData__Sequence__copy(
  const custom_interfaces__msg__SerialData__Sequence * input,
  custom_interfaces__msg__SerialData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__SerialData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__SerialData * data =
      (custom_interfaces__msg__SerialData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__SerialData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__SerialData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__SerialData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
