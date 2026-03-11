// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from vs_msgs:msg/ConeLocationPixel.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "vs_msgs/msg/detail/cone_location_pixel__struct.h"
#include "vs_msgs/msg/detail/cone_location_pixel__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace vs_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _ConeLocationPixel_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ConeLocationPixel_type_support_ids_t;

static const _ConeLocationPixel_type_support_ids_t _ConeLocationPixel_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ConeLocationPixel_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ConeLocationPixel_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ConeLocationPixel_type_support_symbol_names_t _ConeLocationPixel_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vs_msgs, msg, ConeLocationPixel)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vs_msgs, msg, ConeLocationPixel)),
  }
};

typedef struct _ConeLocationPixel_type_support_data_t
{
  void * data[2];
} _ConeLocationPixel_type_support_data_t;

static _ConeLocationPixel_type_support_data_t _ConeLocationPixel_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ConeLocationPixel_message_typesupport_map = {
  2,
  "vs_msgs",
  &_ConeLocationPixel_message_typesupport_ids.typesupport_identifier[0],
  &_ConeLocationPixel_message_typesupport_symbol_names.symbol_name[0],
  &_ConeLocationPixel_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ConeLocationPixel_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ConeLocationPixel_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace vs_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, vs_msgs, msg, ConeLocationPixel)() {
  return &::vs_msgs::msg::rosidl_typesupport_c::ConeLocationPixel_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
