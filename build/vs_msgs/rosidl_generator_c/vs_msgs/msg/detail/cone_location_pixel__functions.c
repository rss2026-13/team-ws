// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vs_msgs:msg/ConeLocationPixel.idl
// generated code does not contain a copyright notice
#include "vs_msgs/msg/detail/cone_location_pixel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vs_msgs__msg__ConeLocationPixel__init(vs_msgs__msg__ConeLocationPixel * msg)
{
  if (!msg) {
    return false;
  }
  // u
  // v
  return true;
}

void
vs_msgs__msg__ConeLocationPixel__fini(vs_msgs__msg__ConeLocationPixel * msg)
{
  if (!msg) {
    return;
  }
  // u
  // v
}

bool
vs_msgs__msg__ConeLocationPixel__are_equal(const vs_msgs__msg__ConeLocationPixel * lhs, const vs_msgs__msg__ConeLocationPixel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // u
  if (lhs->u != rhs->u) {
    return false;
  }
  // v
  if (lhs->v != rhs->v) {
    return false;
  }
  return true;
}

bool
vs_msgs__msg__ConeLocationPixel__copy(
  const vs_msgs__msg__ConeLocationPixel * input,
  vs_msgs__msg__ConeLocationPixel * output)
{
  if (!input || !output) {
    return false;
  }
  // u
  output->u = input->u;
  // v
  output->v = input->v;
  return true;
}

vs_msgs__msg__ConeLocationPixel *
vs_msgs__msg__ConeLocationPixel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vs_msgs__msg__ConeLocationPixel * msg = (vs_msgs__msg__ConeLocationPixel *)allocator.allocate(sizeof(vs_msgs__msg__ConeLocationPixel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vs_msgs__msg__ConeLocationPixel));
  bool success = vs_msgs__msg__ConeLocationPixel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vs_msgs__msg__ConeLocationPixel__destroy(vs_msgs__msg__ConeLocationPixel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vs_msgs__msg__ConeLocationPixel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vs_msgs__msg__ConeLocationPixel__Sequence__init(vs_msgs__msg__ConeLocationPixel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vs_msgs__msg__ConeLocationPixel * data = NULL;

  if (size) {
    data = (vs_msgs__msg__ConeLocationPixel *)allocator.zero_allocate(size, sizeof(vs_msgs__msg__ConeLocationPixel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vs_msgs__msg__ConeLocationPixel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vs_msgs__msg__ConeLocationPixel__fini(&data[i - 1]);
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
vs_msgs__msg__ConeLocationPixel__Sequence__fini(vs_msgs__msg__ConeLocationPixel__Sequence * array)
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
      vs_msgs__msg__ConeLocationPixel__fini(&array->data[i]);
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

vs_msgs__msg__ConeLocationPixel__Sequence *
vs_msgs__msg__ConeLocationPixel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vs_msgs__msg__ConeLocationPixel__Sequence * array = (vs_msgs__msg__ConeLocationPixel__Sequence *)allocator.allocate(sizeof(vs_msgs__msg__ConeLocationPixel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vs_msgs__msg__ConeLocationPixel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vs_msgs__msg__ConeLocationPixel__Sequence__destroy(vs_msgs__msg__ConeLocationPixel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vs_msgs__msg__ConeLocationPixel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vs_msgs__msg__ConeLocationPixel__Sequence__are_equal(const vs_msgs__msg__ConeLocationPixel__Sequence * lhs, const vs_msgs__msg__ConeLocationPixel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vs_msgs__msg__ConeLocationPixel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vs_msgs__msg__ConeLocationPixel__Sequence__copy(
  const vs_msgs__msg__ConeLocationPixel__Sequence * input,
  vs_msgs__msg__ConeLocationPixel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vs_msgs__msg__ConeLocationPixel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vs_msgs__msg__ConeLocationPixel * data =
      (vs_msgs__msg__ConeLocationPixel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vs_msgs__msg__ConeLocationPixel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vs_msgs__msg__ConeLocationPixel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vs_msgs__msg__ConeLocationPixel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
