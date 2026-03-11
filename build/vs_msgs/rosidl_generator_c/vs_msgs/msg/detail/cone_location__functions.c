// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vs_msgs:msg/ConeLocation.idl
// generated code does not contain a copyright notice
#include "vs_msgs/msg/detail/cone_location__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vs_msgs__msg__ConeLocation__init(vs_msgs__msg__ConeLocation * msg)
{
  if (!msg) {
    return false;
  }
  // x_pos
  // y_pos
  return true;
}

void
vs_msgs__msg__ConeLocation__fini(vs_msgs__msg__ConeLocation * msg)
{
  if (!msg) {
    return;
  }
  // x_pos
  // y_pos
}

bool
vs_msgs__msg__ConeLocation__are_equal(const vs_msgs__msg__ConeLocation * lhs, const vs_msgs__msg__ConeLocation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_pos
  if (lhs->x_pos != rhs->x_pos) {
    return false;
  }
  // y_pos
  if (lhs->y_pos != rhs->y_pos) {
    return false;
  }
  return true;
}

bool
vs_msgs__msg__ConeLocation__copy(
  const vs_msgs__msg__ConeLocation * input,
  vs_msgs__msg__ConeLocation * output)
{
  if (!input || !output) {
    return false;
  }
  // x_pos
  output->x_pos = input->x_pos;
  // y_pos
  output->y_pos = input->y_pos;
  return true;
}

vs_msgs__msg__ConeLocation *
vs_msgs__msg__ConeLocation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vs_msgs__msg__ConeLocation * msg = (vs_msgs__msg__ConeLocation *)allocator.allocate(sizeof(vs_msgs__msg__ConeLocation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vs_msgs__msg__ConeLocation));
  bool success = vs_msgs__msg__ConeLocation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vs_msgs__msg__ConeLocation__destroy(vs_msgs__msg__ConeLocation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vs_msgs__msg__ConeLocation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vs_msgs__msg__ConeLocation__Sequence__init(vs_msgs__msg__ConeLocation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vs_msgs__msg__ConeLocation * data = NULL;

  if (size) {
    data = (vs_msgs__msg__ConeLocation *)allocator.zero_allocate(size, sizeof(vs_msgs__msg__ConeLocation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vs_msgs__msg__ConeLocation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vs_msgs__msg__ConeLocation__fini(&data[i - 1]);
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
vs_msgs__msg__ConeLocation__Sequence__fini(vs_msgs__msg__ConeLocation__Sequence * array)
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
      vs_msgs__msg__ConeLocation__fini(&array->data[i]);
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

vs_msgs__msg__ConeLocation__Sequence *
vs_msgs__msg__ConeLocation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vs_msgs__msg__ConeLocation__Sequence * array = (vs_msgs__msg__ConeLocation__Sequence *)allocator.allocate(sizeof(vs_msgs__msg__ConeLocation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vs_msgs__msg__ConeLocation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vs_msgs__msg__ConeLocation__Sequence__destroy(vs_msgs__msg__ConeLocation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vs_msgs__msg__ConeLocation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vs_msgs__msg__ConeLocation__Sequence__are_equal(const vs_msgs__msg__ConeLocation__Sequence * lhs, const vs_msgs__msg__ConeLocation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vs_msgs__msg__ConeLocation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vs_msgs__msg__ConeLocation__Sequence__copy(
  const vs_msgs__msg__ConeLocation__Sequence * input,
  vs_msgs__msg__ConeLocation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vs_msgs__msg__ConeLocation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vs_msgs__msg__ConeLocation * data =
      (vs_msgs__msg__ConeLocation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vs_msgs__msg__ConeLocation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vs_msgs__msg__ConeLocation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vs_msgs__msg__ConeLocation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
