// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from marvelmind_ros2_msgs:msg/HedgeImuFusion.idl
// generated code does not contain a copyright notice
#include "marvelmind_ros2_msgs/msg/detail/hedge_imu_fusion__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
marvelmind_ros2_msgs__msg__HedgeImuFusion__init(marvelmind_ros2_msgs__msg__HedgeImuFusion * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp_ms
  // x_m
  // y_m
  // z_m
  // qw
  // qx
  // qy
  // qz
  // vx
  // vy
  // vz
  // ax
  // ay
  // az
  return true;
}

void
marvelmind_ros2_msgs__msg__HedgeImuFusion__fini(marvelmind_ros2_msgs__msg__HedgeImuFusion * msg)
{
  if (!msg) {
    return;
  }
  // timestamp_ms
  // x_m
  // y_m
  // z_m
  // qw
  // qx
  // qy
  // qz
  // vx
  // vy
  // vz
  // ax
  // ay
  // az
}

bool
marvelmind_ros2_msgs__msg__HedgeImuFusion__are_equal(const marvelmind_ros2_msgs__msg__HedgeImuFusion * lhs, const marvelmind_ros2_msgs__msg__HedgeImuFusion * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp_ms
  if (lhs->timestamp_ms != rhs->timestamp_ms) {
    return false;
  }
  // x_m
  if (lhs->x_m != rhs->x_m) {
    return false;
  }
  // y_m
  if (lhs->y_m != rhs->y_m) {
    return false;
  }
  // z_m
  if (lhs->z_m != rhs->z_m) {
    return false;
  }
  // qw
  if (lhs->qw != rhs->qw) {
    return false;
  }
  // qx
  if (lhs->qx != rhs->qx) {
    return false;
  }
  // qy
  if (lhs->qy != rhs->qy) {
    return false;
  }
  // qz
  if (lhs->qz != rhs->qz) {
    return false;
  }
  // vx
  if (lhs->vx != rhs->vx) {
    return false;
  }
  // vy
  if (lhs->vy != rhs->vy) {
    return false;
  }
  // vz
  if (lhs->vz != rhs->vz) {
    return false;
  }
  // ax
  if (lhs->ax != rhs->ax) {
    return false;
  }
  // ay
  if (lhs->ay != rhs->ay) {
    return false;
  }
  // az
  if (lhs->az != rhs->az) {
    return false;
  }
  return true;
}

bool
marvelmind_ros2_msgs__msg__HedgeImuFusion__copy(
  const marvelmind_ros2_msgs__msg__HedgeImuFusion * input,
  marvelmind_ros2_msgs__msg__HedgeImuFusion * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp_ms
  output->timestamp_ms = input->timestamp_ms;
  // x_m
  output->x_m = input->x_m;
  // y_m
  output->y_m = input->y_m;
  // z_m
  output->z_m = input->z_m;
  // qw
  output->qw = input->qw;
  // qx
  output->qx = input->qx;
  // qy
  output->qy = input->qy;
  // qz
  output->qz = input->qz;
  // vx
  output->vx = input->vx;
  // vy
  output->vy = input->vy;
  // vz
  output->vz = input->vz;
  // ax
  output->ax = input->ax;
  // ay
  output->ay = input->ay;
  // az
  output->az = input->az;
  return true;
}

marvelmind_ros2_msgs__msg__HedgeImuFusion *
marvelmind_ros2_msgs__msg__HedgeImuFusion__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  marvelmind_ros2_msgs__msg__HedgeImuFusion * msg = (marvelmind_ros2_msgs__msg__HedgeImuFusion *)allocator.allocate(sizeof(marvelmind_ros2_msgs__msg__HedgeImuFusion), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(marvelmind_ros2_msgs__msg__HedgeImuFusion));
  bool success = marvelmind_ros2_msgs__msg__HedgeImuFusion__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
marvelmind_ros2_msgs__msg__HedgeImuFusion__destroy(marvelmind_ros2_msgs__msg__HedgeImuFusion * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    marvelmind_ros2_msgs__msg__HedgeImuFusion__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__init(marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  marvelmind_ros2_msgs__msg__HedgeImuFusion * data = NULL;

  if (size) {
    data = (marvelmind_ros2_msgs__msg__HedgeImuFusion *)allocator.zero_allocate(size, sizeof(marvelmind_ros2_msgs__msg__HedgeImuFusion), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = marvelmind_ros2_msgs__msg__HedgeImuFusion__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        marvelmind_ros2_msgs__msg__HedgeImuFusion__fini(&data[i - 1]);
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
marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__fini(marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * array)
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
      marvelmind_ros2_msgs__msg__HedgeImuFusion__fini(&array->data[i]);
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

marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence *
marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * array = (marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence *)allocator.allocate(sizeof(marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__destroy(marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__are_equal(const marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * lhs, const marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!marvelmind_ros2_msgs__msg__HedgeImuFusion__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence__copy(
  const marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * input,
  marvelmind_ros2_msgs__msg__HedgeImuFusion__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(marvelmind_ros2_msgs__msg__HedgeImuFusion);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    marvelmind_ros2_msgs__msg__HedgeImuFusion * data =
      (marvelmind_ros2_msgs__msg__HedgeImuFusion *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!marvelmind_ros2_msgs__msg__HedgeImuFusion__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          marvelmind_ros2_msgs__msg__HedgeImuFusion__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!marvelmind_ros2_msgs__msg__HedgeImuFusion__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
