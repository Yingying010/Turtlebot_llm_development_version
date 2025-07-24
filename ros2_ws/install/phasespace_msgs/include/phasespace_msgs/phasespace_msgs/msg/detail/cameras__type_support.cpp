// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from phasespace_msgs:msg/Cameras.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "phasespace_msgs/msg/detail/cameras__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace phasespace_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Cameras_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) phasespace_msgs::msg::Cameras(_init);
}

void Cameras_fini_function(void * message_memory)
{
  auto typed_message = static_cast<phasespace_msgs::msg::Cameras *>(message_memory);
  typed_message->~Cameras();
}

size_t size_function__Cameras__cameras(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<phasespace_msgs::msg::Camera> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Cameras__cameras(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<phasespace_msgs::msg::Camera> *>(untyped_member);
  return &member[index];
}

void * get_function__Cameras__cameras(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<phasespace_msgs::msg::Camera> *>(untyped_member);
  return &member[index];
}

void fetch_function__Cameras__cameras(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const phasespace_msgs::msg::Camera *>(
    get_const_function__Cameras__cameras(untyped_member, index));
  auto & value = *reinterpret_cast<phasespace_msgs::msg::Camera *>(untyped_value);
  value = item;
}

void assign_function__Cameras__cameras(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<phasespace_msgs::msg::Camera *>(
    get_function__Cameras__cameras(untyped_member, index));
  const auto & value = *reinterpret_cast<const phasespace_msgs::msg::Camera *>(untyped_value);
  item = value;
}

void resize_function__Cameras__cameras(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<phasespace_msgs::msg::Camera> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Cameras_message_member_array[1] = {
  {
    "cameras",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<phasespace_msgs::msg::Camera>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(phasespace_msgs::msg::Cameras, cameras),  // bytes offset in struct
    nullptr,  // default value
    size_function__Cameras__cameras,  // size() function pointer
    get_const_function__Cameras__cameras,  // get_const(index) function pointer
    get_function__Cameras__cameras,  // get(index) function pointer
    fetch_function__Cameras__cameras,  // fetch(index, &value) function pointer
    assign_function__Cameras__cameras,  // assign(index, value) function pointer
    resize_function__Cameras__cameras  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Cameras_message_members = {
  "phasespace_msgs::msg",  // message namespace
  "Cameras",  // message name
  1,  // number of fields
  sizeof(phasespace_msgs::msg::Cameras),
  Cameras_message_member_array,  // message members
  Cameras_init_function,  // function to initialize message memory (memory has to be allocated)
  Cameras_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Cameras_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Cameras_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace phasespace_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<phasespace_msgs::msg::Cameras>()
{
  return &::phasespace_msgs::msg::rosidl_typesupport_introspection_cpp::Cameras_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, phasespace_msgs, msg, Cameras)() {
  return &::phasespace_msgs::msg::rosidl_typesupport_introspection_cpp::Cameras_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
